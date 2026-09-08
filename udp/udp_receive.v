//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 udp_receive
// 【功能描述】 以太网UDP接收模块（输出接收IP、MAC、port）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.START状态读取UDP协议bram缓存，为了解决读潜伏期2拍
// 【说明】 3.HEADER状态读取以太网帧头14字节固定长度。
// 【说明】 4.DATAPACK包结构有IP包头20字节固定和UDP包N字节
// 【说明】 5.UDP接收没有必要做累加和校验。
// 【说明】 6.UDP接收到主机请求，ip、mac、prot输出发送就绪。
// 【说明】 7.udpr_pack_start和udpr_pack_stop通过接收特定数据保存数据
// 【说明】 8.udp接收数据处理建议在这个模块内部实现，高效简洁。
//
//////////////////////////////////////////////////////////////////////////////////
module udp_receive(
	input				 			udpr_clk,						// udpr时钟 
	input        			udpr_rst,						// udpr复位
	input  		 [31:0] udpr_loc_ip,				// udpr本地IP
	input  		 [47:0] udpr_loc_mac,				// udpr本地mac
	input      [15:0] udpr_loc_port,			// udpr本地端口
	output reg [31:0] udpr_dst_ip,				// udpr源IP
	output reg [47:0] udpr_dst_mac,				// udpr源mac
	output reg [15:0] udpr_dst_port,			// udpr源端口
	input             udpr_read_start,		// udpr读启动
	input  		 [ 7:0] udpr_read_data,			// udpr读数据
	input      [10:0] udpr_read_len,			// udpr读长度
	output reg [10:0] udpr_read_addr,			// udpr读计数
	output            udpr_write_clk,     // udpr写时钟
	output reg 				udpr_write_req,			// udpr写请求
	input             udpr_write_ack,			// udpr写响应
	output reg [ 7:0] udpr_write_data,	  // udpr写数据
	output reg [ 8:0] udpr_write_len,	  	// udpr写长度
	output reg [ 8:0] udpr_write_cnt,	  	// udpr写计数
	output reg [ 8:0] udpr_write_offset,	// udpr写偏移
	output reg [23:0] udpr_write_addr,		// udpr写地址sdram
	output reg        udpr_write_en,			// udpr写使能
	output reg        udpr_write_sel,			// udpr写同步缓冲区乒乓
	output reg        udpr_send_ready,		// udpr发送就绪
	output reg   			udpr_read_done			// udpr读完成
);

// udp 状态机定义  
localparam STATE_IDLE  		= 5'b00001;
localparam STATE_START    = 5'b00010;
localparam STATE_HEADER  	= 5'b00100;
localparam STATE_DATAPACK = 5'b01000;
localparam STATE_DONE 		= 5'b10000;

// udp 包参数
localparam PROTOCOL_UDP   = 16'h0800; 
localparam PROTOCOL_IP    = 16'h0011; 

// udp参数
reg [ 4:0] udpr_state;
reg [ 4:0] udpr_state_next;
reg        udpr_read_start_d1;
reg        udpr_read_start_d2;
reg [15:0] udpr_read_loc_port;
reg [31:0] udpr_read_loc_ip;
reg [47:0] udpr_read_loc_mac;
reg [31:0] udpr_read_dst_ip;
reg [47:0] udpr_read_dst_mac;
reg [15:0] udpr_read_dst_port;
reg [10:0] udpr_read_len_offset;
reg        udpr_read_en;
reg [ 2:0] udpr_start_cnt;
reg        udpr_start_done;
reg        udpr_header_done;
reg [ 3:0] udpr_header_cnt;
reg [15:0] udpr_header_type;
reg        udpr_header_error;
reg [10:0] udpr_datapack_rcnt;
reg [10:0] udpr_datapack_len;
reg        udpr_datapack_done;
reg        udpr_datapack_error;
reg [15:0] udpr_data_len;
reg [15:0] udpr_datapack_tl;
reg [ 7:0] udpr_datapack_tt;
reg [ 7:0] udpr_datapack_pl;
reg        udpr_error_pl;
reg        udpr_error_loc_ip;
reg        udpr_error_loc_mac;
reg        udpr_error_loc_port;
reg        udpr_write_en_d1;
reg        udpr_error_stage1;
reg        udpr_write_syn;
reg        udpr_write_start;
reg        udpr_write_end; 
reg        udpr_write_busy;
reg        udpr_index_syn; 
reg [ 9:0] udpr_pack_max;
reg [ 9:0] udpr_pack_nbr; 
reg [31:0] udpr_pack_buf;
reg        udpr_pack_start;
reg        udpr_pack_stop;

//
assign udpr_write_clk = udpr_clk;

// 寄存器
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_state <= STATE_IDLE;
	else
		udpr_state <= udpr_state_next;
end

// 状态跳转
always @(*) begin
	case(udpr_state)
		STATE_IDLE: begin
			if(udpr_read_start_d1)
				udpr_state_next = STATE_START;
			else
				udpr_state_next = STATE_IDLE;
		end
		STATE_START: begin
			if(udpr_start_done)
				udpr_state_next = STATE_HEADER;
			else
				udpr_state_next = STATE_START;
		end
		STATE_HEADER: begin
			if(udpr_header_error)
				udpr_state_next = STATE_IDLE;
			else if(udpr_header_done)
				udpr_state_next = STATE_DATAPACK;
			else
				udpr_state_next = STATE_HEADER;
		end
		STATE_DATAPACK: begin
			if(udpr_datapack_done)
				udpr_state_next = STATE_DONE;
			else if(udpr_datapack_error)
				udpr_state_next = STATE_IDLE;
			else
				udpr_state_next = STATE_DATAPACK;
		end
		STATE_DONE: begin
			udpr_state_next = STATE_IDLE;
		end
		default: udpr_state_next = STATE_IDLE;
	endcase
end

// udp读启动打拍
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0) begin
		udpr_read_start_d1 <= 1'b0;
		udpr_read_start_d2 <= 1'b0;
	end
	else begin 
		udpr_read_start_d1 <= udpr_read_start;
		udpr_read_start_d2 <= udpr_read_start_d1;
	end
end

// 一级锁存读长度,二级跨时钟域隔离减
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_read_len_offset <= 11'd0;
	else if(udpr_read_start_d1)
		udpr_read_len_offset <= udpr_read_len;
	else if(udpr_read_start_d2)
		udpr_read_len_offset <= udpr_read_len_offset - 11'd14;
end

// udp读启动计数
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_start_cnt <= 3'd0;
	else if(udpr_state == STATE_START)
		udpr_start_cnt <= udpr_start_cnt + 1'b1;
	else
		udpr_start_cnt <= 3'd0;
end

// udp读启动完成
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_start_done <= 1'b0;
	else if(udpr_start_cnt == 3'd1)
		udpr_start_done <= 1'b1;
	else
		udpr_start_done <= 1'b0;
end

// 读数据使能
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_read_en <= 1'b0;
	else if(udpr_state == STATE_IDLE)
		udpr_read_en <= 1'b0;
	else if(udpr_state == STATE_START)
		udpr_read_en <= 1'b1;
end

// 读取BRAM地址计数
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_read_addr <= 11'd0;
	else if(udpr_read_en)
		udpr_read_addr <= udpr_read_addr + 1'b1;
	else
		udpr_read_addr <= 11'd0;
end

// 以太网帧头计数器
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_header_cnt <= 4'd0;
	else if(udpr_state == STATE_HEADER)
		udpr_header_cnt <= udpr_header_cnt + 1'b1;
	else
		udpr_header_cnt <= 4'd0;
end

// 提取预测以太网帧头读取完成
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_header_done <= 1'b0;
	else if(udpr_header_cnt == 4'd12)
		udpr_header_done <= 1'b1;
	else
		udpr_header_done <= 1'b0;
end

// 以太网帧无效
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_header_error <= 1'b0;
	else if(udpr_header_cnt == 4'd12)
		udpr_header_error <= udpr_error_loc_mac;
	else
		udpr_header_error <= 1'b0;
end

// 以太网帧头14字节提取
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0) begin
		udpr_read_loc_mac <= 32'd0;
		udpr_read_dst_mac <= 48'd0;
		udpr_header_type  <= 2'd0;
	end
	else if(udpr_state == STATE_HEADER) begin
		case(udpr_header_cnt)
			4'd0:  udpr_read_loc_mac[47:40] <= udpr_read_data;		
			4'd1:  udpr_read_loc_mac[39:32] <= udpr_read_data;
			4'd2:  udpr_read_loc_mac[31:24] <= udpr_read_data;
			4'd3:  udpr_read_loc_mac[23:16] <= udpr_read_data;
			4'd4:  udpr_read_loc_mac[15: 8] <= udpr_read_data;
			4'd5:  udpr_read_loc_mac[ 7: 0] <= udpr_read_data;
			4'd6:  udpr_read_dst_mac[47:40] <= udpr_read_data;
			4'd7:  udpr_read_dst_mac[39:32] <= udpr_read_data;
			4'd8:  udpr_read_dst_mac[31:24] <= udpr_read_data;
			4'd9:  udpr_read_dst_mac[23:16] <= udpr_read_data;
			4'd10: udpr_read_dst_mac[15: 8] <= udpr_read_data;
			4'd11: udpr_read_dst_mac[ 7: 0] <= udpr_read_data;
			4'd12: udpr_header_type [15: 8] <= udpr_read_data;
			4'd13: udpr_header_type [ 7: 0] <= udpr_read_data;
		endcase
	end
end

// 以太网udp包大小
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_datapack_len <= 11'd0;
	else if(udpr_state == STATE_DATAPACK)
		udpr_datapack_len <= udpr_datapack_len - 1'b1;
	else
		udpr_datapack_len <= udpr_read_len_offset;
end

// 以太网udp包计数
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_datapack_rcnt <= 11'd0;
	else if(udpr_state == STATE_DATAPACK)
		udpr_datapack_rcnt <= udpr_datapack_rcnt + 1'b1;
	else 
		udpr_datapack_rcnt <= 11'd0;
end

// 提前预测以太网udp包 IP头20+udp 头8个计数完成
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_datapack_done <= 1'b0;
	else if(udpr_datapack_len == 11'd2)
		udpr_datapack_done <= 1'b1;
	else
		udpr_datapack_done <= 1'b0;
end

// 以太网udp包异常
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_datapack_error <= 1'b0;
	else if(udpr_state == STATE_DATAPACK)
		udpr_datapack_error <= udpr_header_type != PROTOCOL_UDP ? 1'b1 : 1'b0;
	else
		udpr_datapack_error <= 1'b0;
end

// 以太网数据包 IP包头20字节 参数
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0) begin
		udpr_datapack_tl <= 16'd0;
		udpr_datapack_pl <= 8'd0;
		udpr_read_dst_ip <= 32'd0;
		udpr_read_loc_ip <= 32'd0;
		udpr_read_loc_port <= 16'd0;
		udpr_data_len <= 16'd0;
	end
	else if(udpr_state == STATE_IDLE)
		udpr_read_loc_ip <= 32'd0;
	else if(udpr_state == STATE_DATAPACK) begin
		case(udpr_datapack_rcnt)
			11'd2:  udpr_datapack_tl[15: 8] <= udpr_read_data;	 		// IP包头总长度
			11'd3:  udpr_datapack_tl[ 7: 0] <= udpr_read_data;	 		// IP包头总长度																							
			11'd9:  udpr_datapack_pl[ 7: 0] <= udpr_read_data;   		// IP包头协议
			11'd12: udpr_read_dst_ip[31:24] <= udpr_read_data;   		// IP包头源IP
			11'd13: udpr_read_dst_ip[23:16] <= udpr_read_data;   		// IP包头源IP
			11'd14: udpr_read_dst_ip[15: 8] <= udpr_read_data;			// IP包头源IP 
			11'd15: udpr_read_dst_ip[ 7: 0] <= udpr_read_data;  		// IP包头源IP
			11'd16: udpr_read_loc_ip[31:24] <= udpr_read_data; 			// IP包头目的IP 
			11'd17: udpr_read_loc_ip[23:16] <= udpr_read_data;  		// IP包头目的IP 
			11'd18: udpr_read_loc_ip[15: 8] <= udpr_read_data;			// IP包头目的IP  
			11'd19: udpr_read_loc_ip[ 7: 0] <= udpr_read_data;  		// IP包头目的IP 
			11'd20: udpr_read_dst_port[15:8]<= udpr_read_data;  		// UDP包源端口
			11'd21: udpr_read_dst_port[ 7:0]<= udpr_read_data;  		// UDP包源端口
			11'd22: udpr_read_loc_port[15:8]<= udpr_read_data;  		// UDP包目的端口
			11'd23: udpr_read_loc_port[ 7:0]<= udpr_read_data;  		// UDP包目的端口
			11'd24: udpr_data_len[15:8]     <= udpr_read_data;      // UDP包数据长度
			11'd25: udpr_data_len[ 7:0]     <= udpr_read_data;      // UDP包数据长度
		endcase
	end
end

// 以太网IP包头目的IP错误
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_error_loc_ip <= 1'b0;
	else if(udpr_state == STATE_START)
		udpr_error_loc_ip <= 1'b1;
	else if(udpr_datapack_rcnt == 11'd20)
		udpr_error_loc_ip <= udpr_read_loc_ip == udpr_loc_ip ? 1'b0 : 1'b1;
end

// 以太网UDP包头目的端口错误
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_error_loc_port <= 1'b0;
	else if(udpr_state == STATE_START)
		udpr_error_loc_port <= 1'b1;
	else if(udpr_datapack_rcnt == 11'd24)
		udpr_error_loc_port <= udpr_read_loc_port == udpr_loc_port ? 1'b0 : 1'b1;
end

// 以太网目的MAC和本地MAC比较
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_error_loc_mac <= 1'b0;
	else if(udpr_header_cnt == 4'd11)
		udpr_error_loc_mac <= udpr_read_loc_mac != udpr_loc_mac ? 1'b1 : 1'b0;
	else
		udpr_error_loc_mac <= 1'b0;
end

// 以太网IP包头协议错误
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_error_pl <= 1'b0;
	else if(udpr_state == STATE_START)
		udpr_error_pl <= 1'b1;
	else if(udpr_datapack_rcnt == 11'd10)
		udpr_error_pl <= udpr_datapack_pl == PROTOCOL_IP ? 1'b0 : 1'b1;
end

// 把异常标识进行流水线
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0) 
		udpr_error_stage1 <= 1'b0;
	else if(udpr_error_pl || udpr_error_loc_ip || udpr_error_loc_port)
		udpr_error_stage1 <= 1'b0;
	else
		udpr_error_stage1 <= 1'b1;
end

// 以太网帧读取完成
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_read_done <= 1'b0;
	else if(udpr_state == STATE_DONE)
		udpr_read_done <= udpr_error_stage1;
	else
		udpr_read_done <= 1'b0;
end

// 输出以太网帧源IP
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_dst_ip <= 32'd0;
	else if(udpr_state == STATE_DONE)
		udpr_dst_ip <= udpr_read_dst_ip;
end

// 输出以太网帧源MAC
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_dst_mac <= 48'd0;
	else if(udpr_state == STATE_DONE)
		udpr_dst_mac <= udpr_read_dst_mac;
end

// 输出以太网帧源端口
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_dst_port <= 16'd0;
	else if(udpr_state == STATE_DONE)
		udpr_dst_port <= udpr_read_dst_port;
end

// udp写bram 使能
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_en <= 1'b0;
	else if(udpr_state == STATE_IDLE)
		udpr_write_en <= 1'b0;
	else if(udpr_datapack_rcnt == 11'd28) // 跳过UDP校验到数据
		udpr_write_en <= 1'b1;
	else if(udpr_state == STATE_DONE)
		udpr_write_en <= 1'b0;
end

// udp写使能打拍
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_en_d1 <= 1'b0;
	else
		udpr_write_en_d1 <= udpr_write_en;
end

// udp数据写入bram
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_data <= 8'd0;
	else 
		udpr_write_data <= udpr_read_data;
end

// udp写地址计数
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_cnt <= 9'd0;
	else if(udpr_write_en)
		udpr_write_cnt <= udpr_write_cnt + 1'b1;
	else
		udpr_write_cnt <= udpr_write_offset;
end

// udp写长度
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_len <= 9'd0;
	else 
		udpr_write_len <= 9'd256;
end

// 写同步信号
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_syn <= 1'b0;
	else if(udpr_pack_start	== 1'b1)
		udpr_write_syn <= 1'b0;
	else if(udpr_write_cnt == 9'd255)
		udpr_write_syn <= ~udpr_write_syn;
	else if(udpr_write_cnt == 9'd511)
		udpr_write_syn <= ~udpr_write_syn;
end

// 写忙信号
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_busy <= 1'b0;
	else if(udpr_write_ack)
		udpr_write_busy <= 1'b0;
	else if(udpr_write_syn != udpr_index_syn)
		udpr_write_busy <= 1'b1;
end

// 写请求
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_req <= 1'b0;
	else if(udpr_write_syn != udpr_index_syn)
		udpr_write_req <= !udpr_write_busy;
end

// 写同步索引双缓冲乒乓索引0_0-255, 1_255-511
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_index_syn <= 1'b0;
	else if(udpr_pack_start == 1'b1)
		udpr_index_syn <= 1'b0;
	else if(udpr_write_ack)
		udpr_index_syn <= ~udpr_index_syn;
end

// 写偏移双缓冲起始地址 0, 256
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_offset <= 9'd0;
	else 
		udpr_write_offset <= {udpr_index_syn, 8'd0};
end

// 写sdram地址
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_addr <= 24'd0;
	else if(udpr_pack_start == 1'b1)
		udpr_write_addr <= {3'b0, ~udpr_write_sel, 20'b0};
	else if(udpr_write_ack)
		udpr_write_addr <= udpr_write_addr + 24'd256;
end

// 拼接32bit数据
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_pack_buf <= 32'd0;
	else if(udpr_write_cnt < 9'd5)
		udpr_pack_buf <= {udpr_pack_buf[23:0], udpr_write_data};
end

// 检测进入传输状态
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_pack_start <= 1'b0;
	else if(udpr_pack_buf == 32'h89ABCDEF)
		udpr_pack_start <= 1'b1;
	else
		udpr_pack_start <= 1'b0;
end

// 检测文件传输完成
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_pack_stop <= 1'b0;
	else if(udpr_pack_buf == 32'hFEDCBA98)
		udpr_pack_stop <= 1'b1;
	else
		udpr_pack_stop <= 1'b0;
end

// 双缓冲切换
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_write_sel <= 1'b0;
	else if(udpr_pack_stop)
		udpr_write_sel <= ~udpr_write_sel;
end

// 发送就绪已经获取远程ip、mac、prot
always @(posedge udpr_clk or negedge udpr_rst) begin
	if(udpr_rst == 1'b0)
		udpr_send_ready <= 1'b0;
	else if(udpr_state == STATE_DONE)
		udpr_send_ready <= 1'b1;
end

//  
endmodule
