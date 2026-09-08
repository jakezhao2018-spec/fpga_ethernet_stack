//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 udp_send
// 【功能描述】 以太网UDP发送模块（封包+累加和校验）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.HEADER状态读取以太网帧头14字节固定长度。
// 【说明】 3.DATAPACK状态包结构有IP包头20字节固定和UDP包N字节
// 【说明】 4.采用边读取bram发送数据，边计算累加和。
// 【说明】 5.为了精简高效IP包和UDP包累加和分开独立计算的。
// 【说明】 6.UDP包累加和计算最麻烦，采用先计算在写入bram。
// 【说明】 7.STATE_CHECK状态负责校验和回填，地址掉头。
// 【说明】 8.STATE_DONE状态负责告诉后端分包完成可以发送。
//
//////////////////////////////////////////////////////////////////////////////////
module udp_send(
	input						 	udpt_clk,					 // udp时钟 
	input 		      	udpt_rst,					 // udp复位
	input      [15:0] udpt_loc_port,		 // udp本地端口
	input 		 [31:0] udpt_loc_ip,			 // udp本地IP
	input 		 [47:0] udpt_loc_mac,			 // udp本地mac
	input 		 [31:0] udpt_dst_ip,			 // udp源IP
	input 		 [47:0] udpt_dst_mac,			 // udp源mac
	input      [15:0] udpt_dst_port,		 // udp源端口
	input      [ 7:0] udpt_read_data,	 	 // udp读数据
	input      [10:0] udpt_read_len,		 // udp读长度
	input      [10:0] udpt_read_offset,	 // udp读偏移
	output reg [10:0] udpt_read_cnt,     // udp读计数
	output reg [23:0] udpt_read_addr,	 	 // udp读sdram地址用
	output            udpt_read_clk,     // udp读取时钟
	input             udpt_write_start,	 // udp写启动
	output reg [ 7:0] udpt_write_data,	 // udp写数据
	output reg [10:0] udpt_write_len,		 // udp写长度
	output reg [10:0] udpt_write_addr,	 // udp写地址
	output reg   			udpt_write_en,		 // udp写使能
	output reg   			udpt_write_done 	 // udp写完成
);

// udp 状态机定义  
localparam STATE_IDLE  		= 6'b000001;
localparam STATE_HEADER  	= 6'b000010;
localparam STATE_IPPACK  	= 6'b000100;
localparam STATE_DATAPACK = 6'b001000;
localparam STATE_CHECK    = 6'b010000;
localparam STATE_DONE     = 6'b100000;

// 以太网udp参数
localparam UDP_PROTOCOL_FT = 16'h0800;  // IP包协议帧类型

// SDRAM主控制器参数
reg [ 5:0] udpt_state;
reg [ 5:0] udpt_state_next;
reg        udpt_write_start_d1;
reg        udpt_write_start_d2;
reg [15:0] udpt_write_loc_port;
reg [31:0] udpt_write_loc_ip;
reg [47:0] udpt_write_loc_mac;
reg [15:0] udpt_write_dst_port;
reg [31:0] udpt_write_dst_ip;
reg [47:0] udpt_write_dst_mac;
reg [ 3:0] udpt_header_cnt;
reg [ 7:0] udpt_header_data;
reg        udpt_header_done;
reg [ 4:0] udpt_ippack_cnt;
reg [ 4:0] udpt_ippack_len;
reg [ 7:0] udpt_ippack_data;
reg [15:0] udpt_ippack_tl;
reg        udpt_ippack_sum_en;
reg        udpt_ippack_sum_init;
reg        udpt_ippack_done;
reg        udpt_ippack_done_d1;
reg        udpt_ippack_done_d2;
reg        udpt_ippack_done_d3;
reg [10:0] udpt_datapack_cnt;
reg [10:0] udpt_datapack_len;
reg [15:0] udpt_datapack_tl;
reg [ 7:0] udpt_datapack_data;
reg        udpt_datapack_done;  
reg        udpt_datapack_done_d1; 
reg [ 7:0] udpt_fakepack_data;
reg [ 4:0] udpt_fakepack_cnt;
reg        udpt_fakepack_en;
reg        udpt_fakepack_en_d1;
reg        udpt_fakepack_init;
reg [ 7:0] udpt_check_data;
reg [ 2:0] udpt_check_cnt; 
reg [ 5:0] udpt_check_addr;
reg        udpt_check_done;
reg        udpt_read_en;
reg [ 7:0] udpt_read_data_d1;
reg [ 7:0] udpt_read_data_d2;
reg [ 7:0] udpt_read_data_d3;
reg        udpt_write_en_d1;
wire[15:0] udpt_ippack_buf;
wire[15:0] udpt_fakepack_buf;

// udp读取时钟
assign udpt_read_clk = udpt_clk;

// 寄存器
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_state <= STATE_IDLE;
	else
		udpt_state <= udpt_state_next;
end

// 状态跳转
always @(*) begin
	case(udpt_state)
		STATE_IDLE: begin
			if(udpt_write_start_d1)
				udpt_state_next = STATE_HEADER;
			else
				udpt_state_next = STATE_IDLE;
		end
		STATE_HEADER: begin
			if(udpt_header_done)
				udpt_state_next = STATE_IPPACK;
			else
				udpt_state_next = STATE_HEADER;
		end
		STATE_IPPACK: begin
			if(udpt_ippack_done)
				udpt_state_next = STATE_DATAPACK;
			else
				udpt_state_next = STATE_IPPACK;
		end
		STATE_DATAPACK: begin
			if(udpt_datapack_done)
				udpt_state_next = STATE_CHECK;
			else	
				udpt_state_next = STATE_DATAPACK;
		end
		STATE_CHECK: begin
			if(udpt_check_done)
				udpt_state_next = STATE_DONE;
			else	
				udpt_state_next = STATE_CHECK;
		end
		STATE_DONE: begin
			udpt_state_next = STATE_IDLE;
		end
		default: udpt_state_next = STATE_IDLE;
	endcase
end

// 写启动打2拍
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) begin
		udpt_write_start_d1 <= 1'b0;
		udpt_write_start_d2 <= 1'b0;
	end
	else begin
		udpt_write_start_d1 <= udpt_write_start;
		udpt_write_start_d2 <= udpt_write_start_d1;
	end
end

// 一级锁存输入源MAC和IP
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) begin
		udpt_write_dst_ip <= 32'd0;
		udpt_write_dst_mac <= 48'd0;
		udpt_write_dst_port <= 16'd0;
	end
	else if(udpt_write_start_d1)begin
		udpt_write_dst_ip <= udpt_dst_ip;
		udpt_write_dst_mac <= udpt_dst_mac;
		udpt_write_dst_port <= udpt_dst_port;
	end
end

// 一级锁存输入本地MAC和IP
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) begin
		udpt_write_loc_ip <= 32'd0;
		udpt_write_loc_mac <= 48'd0;
		udpt_write_loc_port <= 16'd0;
	end
	else if(udpt_write_start_d1)begin
		udpt_write_loc_ip <= udpt_loc_ip;
		udpt_write_loc_mac <= udpt_loc_mac;
		udpt_write_loc_port <= udpt_loc_port;
	end
end

// 一级锁存IP头总长度,隔离跨时钟域二级加法
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) 
		udpt_ippack_tl <= 16'd0;
	else if(udpt_write_start_d1) 
		udpt_ippack_tl <= udpt_read_len;
	else if(udpt_write_start_d2)
		udpt_ippack_tl <= udpt_ippack_tl + 11'd28;
end

// 一级锁存udp包返回参数长度，隔离跨时钟域二级加法
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) 
		udpt_datapack_tl <= 11'd0;
	else if(udpt_write_start_d1) 
		udpt_datapack_tl <= udpt_read_len;
	else if(udpt_write_start_d2) 
		udpt_datapack_tl <= udpt_datapack_tl + 11'd8;
end

// 以太网帧头计数(0-13)
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_header_cnt <= 4'd0;
	else if(udpt_state == STATE_HEADER)
		udpt_header_cnt <= udpt_header_cnt + 1'b1;
	else
		udpt_header_cnt <= 4'd0;
end

// 以太网帧头14字节数据
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_header_data <= 8'd0;
	else begin
		case(udpt_header_cnt)
			4'd0:  udpt_header_data <= udpt_write_dst_mac[47:40]; 		 // 目的MAC地址
			4'd1:  udpt_header_data <= udpt_write_dst_mac[39:32]; 		 // 目的MAC地址
			4'd2:  udpt_header_data <= udpt_write_dst_mac[31:24]; 		 // 目的MAC地址
			4'd3:  udpt_header_data <= udpt_write_dst_mac[23:16]; 		 // 目的MAC地址
			4'd4:  udpt_header_data <= udpt_write_dst_mac[15: 8]; 		 // 目的MAC地址
			4'd5:  udpt_header_data <= udpt_write_dst_mac[ 7: 0]; 		 // 目的MAC地址
			4'd6:  udpt_header_data <= udpt_write_loc_mac[47:40]; 		 // 源MAC地址
			4'd7:  udpt_header_data <= udpt_write_loc_mac[39:32]; 		 // 源MAC地址
			4'd8:  udpt_header_data <= udpt_write_loc_mac[31:24]; 		 // 源MAC地址
			4'd9:  udpt_header_data <= udpt_write_loc_mac[23:16]; 		 // 源MAC地址
			4'd10: udpt_header_data <= udpt_write_loc_mac[15: 8]; 		 // 源MAC地址
			4'd11: udpt_header_data <= udpt_write_loc_mac[ 7: 0]; 		 // 源MAC地址
			4'd12: udpt_header_data <= UDP_PROTOCOL_FT[15:8];		 		 	 // 协议类型
			4'd13: udpt_header_data <= UDP_PROTOCOL_FT[ 7:0];		 		 	 // 协议类型
			default: udpt_header_data <= 8'd0;
		endcase
	end
end

// 以太网帧头计数完成
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_header_done <= 1'b0;
	else if(udpt_header_cnt == 4'd13)
		udpt_header_done <= 1'b1;
	else
		udpt_header_done <= 1'b0;
end

// 以太网帧IP包头计数
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_ippack_cnt <= 5'd0;
	else if(udpt_state == STATE_IPPACK || udpt_header_done)
		udpt_ippack_cnt <= udpt_ippack_cnt + 1'b1;
	else
		udpt_ippack_cnt <= 5'd0;
end

// 以太网帧IP包数据20字节填充
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_ippack_data <= 8'd0;
	else begin
		case(udpt_ippack_cnt)
			5'd0:  udpt_ippack_data <= 8'h45; 		 									// IP包头版本+帧头
			5'd1:  udpt_ippack_data <= 8'h00; 		 									// IP包头服务类型
			5'd2:  udpt_ippack_data <= udpt_ippack_tl[15:8]; 				// IP包头总长度
			5'd3:  udpt_ippack_data <= udpt_ippack_tl[ 7:0]; 				// IP包头总长度
			5'd4:  udpt_ippack_data <= 8'h00;   	 									// IP包头标识
			5'd5:  udpt_ippack_data <= 8'h00;	 	 										// IP包头标识
			5'd6:  udpt_ippack_data <= 8'h00; 		 									// IP包头标志+偏移
			5'd7:  udpt_ippack_data <= 8'h00; 		 									// IP包头标志+偏移
			5'd8:  udpt_ippack_data <= 8'h80; 		 									// IP包头TTL
			5'd9:  udpt_ippack_data <= 8'h11;  	 										// IP包头协议
			5'd10: udpt_ippack_data <= 8'h00;		 	 									// IP包头校验
			5'd11: udpt_ippack_data <= 8'h00; 	 		 								// IP包头校验
			5'd12: udpt_ippack_data <= udpt_write_loc_ip[31:24];	 	// IP包头源IP地址
			5'd13: udpt_ippack_data <= udpt_write_loc_ip[23:16];	 	// IP包头源IP地址
			5'd14: udpt_ippack_data <= udpt_write_loc_ip[15: 8];	 	// IP包头源IP地址
			5'd15: udpt_ippack_data <= udpt_write_loc_ip[ 7: 0];	 	// IP包头源IP地址
			5'd16: udpt_ippack_data <= udpt_write_dst_ip[31:24];	 	// IP包头目的IP地址
			5'd17: udpt_ippack_data <= udpt_write_dst_ip[23:16];	 	// IP包头目的IP地址
			5'd18: udpt_ippack_data <= udpt_write_dst_ip[15: 8];	 	// IP包头目的IP地址
			5'd19: udpt_ippack_data <= udpt_write_dst_ip[ 7: 0];	 	// IP包头目的IP地址
			default: udpt_ippack_data <= 8'h00;					  
		endcase
	end
end

// 以太网帧IP包头倒计时
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_ippack_len <= 5'd0;
	else if(udpt_state == STATE_IPPACK)
		udpt_ippack_len <= udpt_ippack_len - 1'b1;
	else
		udpt_ippack_len <= 5'd20;
end

// 以太网帧IP包头计算完成
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_ippack_done <= 1'b0;
	else if(udpt_ippack_len == 5'd2)
		udpt_ippack_done <= 1'b1;
	else
		udpt_ippack_done <= 1'b0;
end

// IP包完成延迟拍加载校验
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) begin
		udpt_ippack_done_d1 <= 1'b0;
		udpt_ippack_done_d2 <= 1'b0;
		udpt_ippack_done_d3 <= 1'b0;
	end
	else begin
		udpt_ippack_done_d1 <= udpt_ippack_done;
		udpt_ippack_done_d2 <= udpt_ippack_done_d1;
		udpt_ippack_done_d3 <= udpt_ippack_done_d2;
	end
end

// 以太网帧IP包头校验使能
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_ippack_sum_en <= 1'b0;
	else if(udpt_header_done)
		udpt_ippack_sum_en <= 1'b1;
	else if(udpt_ippack_done_d3)
		udpt_ippack_sum_en <= 1'b0;
end

// 以太网帧IP包头校验初始化
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_ippack_sum_init <= 1'b0;
	else if(udpt_state == STATE_IDLE)
		udpt_ippack_sum_init <= 1'b1;
	else 
		udpt_ippack_sum_init <= 1'b0;
end

// 以太网帧UDP包头+数据计数
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_datapack_cnt <= 11'd0;
	else if(udpt_state == STATE_DATAPACK || udpt_ippack_done)
		udpt_datapack_cnt <= udpt_datapack_cnt + 1'b1;
	else
		udpt_datapack_cnt <= 11'd0;
end

// 以太网帧UDP包长度倒计时
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_datapack_len <= 11'd0;
	else if(udpt_state == STATE_DATAPACK)
		udpt_datapack_len <= udpt_datapack_len - 1'b1;
	else
		udpt_datapack_len <= udpt_datapack_tl[10:0];
end

// udp读bram使能
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_read_en <= 1'b0;
	else if(udpt_state == STATE_IDLE)
		udpt_read_en <= 1'b0;
	else if(udpt_datapack_cnt == 11'd2)
		udpt_read_en <= 1'b1;
	else if(udpt_datapack_len == 11'd5)
		udpt_read_en <= 1'b0;
end

// udp读取发送参数
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_read_cnt <= 11'd0;
	else if(udpt_read_en)
		udpt_read_cnt <= udpt_read_cnt + 1'b1;
	else
		udpt_read_cnt <= udpt_read_offset;
end

// udp读取发送参数打2拍提前输出
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) begin
		udpt_read_data_d1 <= 8'd0;
		udpt_read_data_d2 <= 8'd0;
		udpt_read_data_d3 <= 8'd0;
	end
	else begin
		udpt_read_data_d1 <= udpt_read_data;
		udpt_read_data_d2 <= udpt_read_data_d1;
		udpt_read_data_d3 <= udpt_read_data_d2;
	end
end

// 以太网帧UDP数据
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_datapack_data <= 8'd0;
	else begin
		case(udpt_datapack_cnt)
			11'd0: udpt_datapack_data <= udpt_write_loc_port[15:8];	 		 // UDP本地端口
			11'd1: udpt_datapack_data <= udpt_write_loc_port[ 7:0];	 		 // UDP本地端口
			11'd2: udpt_datapack_data <= udpt_write_dst_port[15:8];	 		 // UDP源端口
			11'd3: udpt_datapack_data <= udpt_write_dst_port[ 7:0];	 		 // UDP源端口
			11'd4: udpt_datapack_data <= udpt_datapack_tl[15:8];	 			 // UDP数据长度
			11'd5: udpt_datapack_data <= udpt_datapack_tl[ 7:0];	 			 // UDP数据长度
			11'd6: udpt_datapack_data <= 8'h00;										 			 // UDP校验
			11'd7: udpt_datapack_data <= 8'h00;										 			 // UDP校验
			default: udpt_datapack_data <= udpt_read_data_d3;					
		endcase
	end
end

// 以太网帧udp包计数完成
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_datapack_done <= 1'b0;
	else if(udpt_datapack_len == 11'd2)
		udpt_datapack_done <= 1'b1;
	else
		udpt_datapack_done <= 1'b0;
end

// 以太网帧udp包计数完成打拍
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_datapack_done_d1 <= 1'b0;
	else 
		udpt_datapack_done_d1 <= udpt_datapack_done;
end

// udp伪协议计数
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_fakepack_cnt <= 5'd0;
	else if(udpt_fakepack_en) begin
		if(udpt_fakepack_cnt < 5'd20)
			udpt_fakepack_cnt <= udpt_fakepack_cnt + 1'b1;
	end
	else
		udpt_fakepack_cnt <= 5'd0;
end

// udp伪协议数据校验
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_fakepack_data <= 8'd0;
	else begin
		case(udpt_fakepack_cnt)
			5'd0:  udpt_fakepack_data <= udpt_write_loc_ip[31:24];  	// 目的IP
			5'd1:  udpt_fakepack_data <= udpt_write_loc_ip[23:16];		// 目的IP
			5'd2:  udpt_fakepack_data <= udpt_write_loc_ip[15: 8];		// 目的IP
			5'd3:  udpt_fakepack_data <= udpt_write_loc_ip[ 7: 0];		// 目的IP
			5'd4:  udpt_fakepack_data <= udpt_write_dst_ip[31:24];  	// 源IP
			5'd5:  udpt_fakepack_data <= udpt_write_dst_ip[23:16];		// 源IP
			5'd6:  udpt_fakepack_data <= udpt_write_dst_ip[15: 8];		// 源IP
			5'd7:  udpt_fakepack_data <= udpt_write_dst_ip[ 7: 0];		// 源IP
			5'd8:  udpt_fakepack_data <= 8'h00;										 		// 协议号
			5'd9:  udpt_fakepack_data <= 8'h11;										 		// 协议号
			5'd10: udpt_fakepack_data <= udpt_datapack_tl[15:8];	 		// udp长度
			5'd11: udpt_fakepack_data <= udpt_datapack_tl[ 7:0];	 		// udp长度
			5'd12: udpt_fakepack_data <= udpt_write_loc_port[15:8];	 	// UDP本地端口
			5'd13: udpt_fakepack_data <= udpt_write_loc_port[ 7:0];	 	// UDP本地端口
			5'd14: udpt_fakepack_data <= udpt_write_dst_port[15:8];	 	// UDP源端口
			5'd15: udpt_fakepack_data <= udpt_write_dst_port[ 7:0];	 	// UDP源端口
			5'd16: udpt_fakepack_data <= udpt_datapack_tl[15:8];	 		// UDP数据长度
			5'd17: udpt_fakepack_data <= udpt_datapack_tl[ 7:0];	 		// UDP数据长度
			5'd18: udpt_fakepack_data <= 8'h00;										 		// UDP校验
			5'd19: udpt_fakepack_data <= 8'h00;										 		// UDP校验
			default: udpt_fakepack_data <= udpt_read_en ? udpt_read_data : 8'd0;	// UDP发送数据
		endcase
	end
end

// udp伪协议校验使能
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_fakepack_en <= 1'b0;
	else if(udpt_ippack_cnt == 5'd4)
		udpt_fakepack_en <= 1'b1;
	else if(udpt_datapack_done)
		udpt_fakepack_en <= 1'b0;
end

// udp伪协议校验初始化
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_fakepack_init <= 1'b0;
	else if(udpt_state == STATE_IDLE)
		udpt_fakepack_init <= 1'b1;
	else 
		udpt_fakepack_init <= 1'b0;
end

// udp伪协议校验使能延迟1拍
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_fakepack_en_d1 <= 1'b0;
	else 
		udpt_fakepack_en_d1 <= udpt_fakepack_en;
end

// 以太网帧udp校验计数
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_check_cnt <= 3'd0;
	else if(udpt_state == STATE_CHECK || udpt_datapack_done)
		udpt_check_cnt <= udpt_check_cnt + 1'b1;
	else
		udpt_check_cnt <= 3'd0;
end

// 以太网IP包头+UDP包校验
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_check_data <= 8'd0;
	else begin
		case(udpt_check_cnt)
			3'd0: udpt_check_data <= udpt_ippack_buf[15:8]; 	// IP包头校验
			3'd1: udpt_check_data <= udpt_ippack_buf[ 7:0]; 	// IP包头校验
			3'd2: udpt_check_data <= udpt_fakepack_buf[15:8]; // UDP包校验
			3'd3: udpt_check_data <= udpt_fakepack_buf[ 7:0]; // UDP包校验
			default: udpt_check_data <= 8'd0;
		endcase
	end
end

// 以太网IP包头+UDP包地址回退
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_check_addr <= 6'd0;
	else begin
		case(udpt_check_cnt)
			3'd0: udpt_check_addr <= 6'd24; 	// IP包头校验地址
			3'd1: udpt_check_addr <= 6'd25; 	// IP包头校验地址
			3'd2: udpt_check_addr <= 6'd40; 	// UDP包头校验地址
			3'd3: udpt_check_addr <= 6'd41; 	// UDP包头校验地址
			default: udpt_check_addr <= 6'd0;
		endcase
	end
end

// 以太网帧udp校验计数完成
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_check_done <= 1'b0;
	else if(udpt_check_cnt == 3'd3)
		udpt_check_done <= 1'b1;
	else
		udpt_check_done <= 1'b0;
end

// bram写使能控制
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_write_en_d1 <= 1'b0;
	else if(udpt_state == STATE_HEADER)
		udpt_write_en_d1 <= 1'b1;
	else if(udpt_check_done)
		udpt_write_en_d1 <= 1'b0;
end

// bram写使能打1拍
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0) 
		udpt_write_en <= 1'b0;
	else 
		udpt_write_en <= udpt_write_en_d1;
end

// bram写地址和数据延迟1拍计数
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_write_addr <= 11'd0;
	else if(udpt_state == STATE_CHECK)
		udpt_write_addr <= {5'b0, udpt_check_addr};
	else if(udpt_write_en)
		 udpt_write_addr <= udpt_write_addr + 1'b1;
	else
		udpt_write_addr <= 11'd0;	
end

// udp以太网帧写入bram
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_write_data <= 8'd0;
	else begin
		case(udpt_state)
			STATE_HEADER: 	udpt_write_data <= udpt_header_data;
			STATE_IPPACK:   udpt_write_data <= udpt_ippack_data;
			STATE_DATAPACK: udpt_write_data <= udpt_datapack_data;
			STATE_CHECK:    udpt_write_data <= udpt_check_data;
			default:        udpt_write_data <= 8'h00;
		endcase
	end
end

// 提前预判udp发送包写入bram完成填补校验延迟
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_write_done <= 1'b0;
	else if(udpt_datapack_done_d1)
		udpt_write_done <= 1'b1;
	else
		udpt_write_done <= 1'b0;
end

// 锁存udp发送包写入长度
always @(posedge udpt_clk or negedge udpt_rst) begin
	if(udpt_rst == 1'b0)
		udpt_write_len <= 11'd0;
	else if(udpt_datapack_done_d1)
		udpt_write_len <= udpt_write_addr + 11'd1;
end

// IP包20字节校验计算
eth_check_sum eth_ippack_sum( 
	.check_clk(udpt_clk),									// sum校验时钟 
	.check_rst(udpt_rst),									// sum校验复位
	.check_sum_in(udpt_ippack_data),			// sum校验输入
	.check_sum_en(udpt_ippack_sum_en),		// sum校验使能
	.check_sum_val(udpt_ippack_sum_init), // sum校验初始化 
	.check_sum_out(udpt_ippack_buf)     	// sum校验输出
);

// UDP包N字节校验计算
eth_check_sum eth_datapack_sum( 
	.check_clk(udpt_clk),									// sum校验时钟 
	.check_rst(udpt_rst),									// sum校验复位
	.check_sum_in(udpt_fakepack_data),	  // sum校验输入
	.check_sum_en(udpt_fakepack_en_d1),	  // sum校验使能
	.check_sum_val(udpt_fakepack_init),		// sum校验初始化 
	.check_sum_out(udpt_fakepack_buf)     // sum校验输出
);

endmodule
