//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 icmp_receive
// 【功能描述】 以太网ICMP接收模块（解包）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.START状态读取ICMP协议bram缓存，为了解决读潜伏期2拍
// 【说明】 3.STATE_HEADER状态读取帧头，比对MAC不是自己丢弃
// 【说明】 4.STATE_DATAPACK状态把ICMP数据请求参数写入bram给后端返回使用。
// 【说明】 5.STATE_DONE状态输出读取完成，告诉后端可以发送了
//
//////////////////////////////////////////////////////////////////////////////////
module icmp_receive(
	input				 			icmpr_clk,					// icmpr时钟 
	input        			icmpr_rst,					// icmpr复位
	input  		 [31:0] icmpr_loc_ip,				// icmpr本地IP
	input  		 [47:0] icmpr_loc_mac,			// icmpr本地mac
	output reg [31:0] icmpr_dst_ip,				// icmpr目标IP
	output reg [47:0] icmpr_dst_mac,			// icmpr目标mac
	input             icmpr_read_start,		// icmpr读启动
	input  		 [ 7:0] icmpr_read_data,		// icmpr读数据
	input      [ 7:0] icmpr_read_len,			// icmpr读长度
	output reg [ 7:0] icmpr_read_addr,		// icmpr读计数
	output reg [ 7:0] icmpr_write_data,	  // icmpr写数据
	output reg [ 7:0] icmpr_write_len,	  // icmpr写长度
	output reg [ 7:0] icmpr_write_addr,	  // icmpr写地址
	output reg        icmpr_write_en,			// icmpr写使能
	output reg   			icmpr_read_done			// icmpr读完成
);

// icmp 状态机定义  
localparam STATE_IDLE  		= 5'b00001;
localparam STATE_START    = 5'b00010;
localparam STATE_HEADER  	= 5'b00100;
localparam STATE_DATAPACK = 5'b01000;
localparam STATE_DONE 		= 5'b10000;

// icmp 包参数
localparam PROTOCOL_ICMP   = 16'h0800; 
localparam PROTOCOL_IP     = 8'h01; 

// ICMP参数
reg [ 4:0] icmpr_state;
reg [ 4:0] icmpr_state_next;
reg [31:0] icmpr_read_loc_ip;
reg [47:0] icmpr_read_loc_mac;
reg [31:0] icmpr_read_dst_ip;
reg [47:0] icmpr_read_dst_mac;
reg        icmpr_read_en;
reg [ 2:0] icmpr_start_cnt;
reg        icmpr_start_done;
reg        icmpr_header_done;
reg [ 3:0] icmpr_header_cnt;
reg [15:0] icmpr_header_type;
reg        icmpr_header_error;
reg [ 7:0] icmpr_datapack_rcnt;
reg [ 4:0] icmpr_datapack_wcnt;
reg [ 7:0] icmpr_datapack_len;
reg        icmpr_datapack_done;
reg        icmpr_datapack_error;
reg [ 4:0] icmpr_datapack_vr;
reg [ 3:0] icmpr_datapack_fl;
reg [ 7:0] icmpr_datapack_st;
reg [15:0] icmpr_datapack_tl;
reg [15:0] icmpr_datapack_if;
reg [ 2:0] icmpr_datapack_lg;
reg [12:0] icmpr_datapack_of;
reg [ 7:0] icmpr_datapack_tt;
reg [ 7:0] icmpr_datapack_pl;
reg [15:0] icmpr_datapack_fc;
reg        icmpr_error_pl;
reg        icmpr_error_loc_ip;
reg        icmpr_error_loc_mac;
reg        icmpr_write_en_d1;

// 寄存器
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_state <= STATE_IDLE;
	else
		icmpr_state <= icmpr_state_next;
end

// 状态跳转
always @(*) begin
	case(icmpr_state)
		STATE_IDLE: begin
			if(icmpr_read_start)
				icmpr_state_next = STATE_START;
			else
				icmpr_state_next = STATE_IDLE;
		end
		STATE_START: begin
			if(icmpr_start_done)
				icmpr_state_next = STATE_HEADER;
			else
				icmpr_state_next = STATE_START;
		end
		STATE_HEADER: begin
			if(icmpr_header_error)
				icmpr_state_next = STATE_IDLE;
			else if(icmpr_header_done)
				icmpr_state_next = STATE_DATAPACK;
			else
				icmpr_state_next = STATE_HEADER;
		end
		STATE_DATAPACK: begin
			if(icmpr_datapack_done)
				icmpr_state_next = STATE_DONE;
			else if(icmpr_datapack_error)
				icmpr_state_next = STATE_IDLE;
			else
				icmpr_state_next = STATE_DATAPACK;
		end
		STATE_DONE: begin
			icmpr_state_next = STATE_IDLE;
		end
		default: icmpr_state_next = STATE_IDLE;
	endcase
end

// icmp读启动计数
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_start_cnt <= 3'd0;
	else if(icmpr_state == STATE_START)
		icmpr_start_cnt <= icmpr_start_cnt + 1'b1;
	else
		icmpr_start_cnt <= 3'd0;
end

// icmp读启动完成
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_start_done <= 1'b0;
	else if(icmpr_start_cnt == 3'd1)
		icmpr_start_done <= 1'b1;
	else
		icmpr_start_done <= 1'b0;
end

// 读数据使能
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_read_en <= 1'b0;
	else if(icmpr_state == STATE_IDLE)
		icmpr_read_en <= 1'b0;
	else if(icmpr_state == STATE_START)
		icmpr_read_en <= 1'b1;
end

// 读取BRAM地址计数
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_read_addr <= 8'd0;
	else if(icmpr_read_en)
		icmpr_read_addr <= icmpr_read_addr + 1'b1;
	else
		icmpr_read_addr <= 8'd0;
end

// 以太网帧头计数器
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_header_cnt <= 4'd0;
	else if(icmpr_state == STATE_HEADER)
		icmpr_header_cnt <= icmpr_header_cnt + 1'b1;
	else
		icmpr_header_cnt <= 4'd0;
end

// 提取预测以太网帧头读取完成
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_header_done <= 1'b0;
	else if(icmpr_header_cnt == 4'd12)
		icmpr_header_done <= 1'b1;
	else
		icmpr_header_done <= 1'b0;
end


// 以太网目的MAC和本地MAC比较
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_error_loc_mac <= 1'b0;
	else if(icmpr_header_cnt == 4'd11)
		icmpr_error_loc_mac <= icmpr_read_loc_mac != icmpr_loc_mac ? 1'b1 : 1'b0;
	else
		icmpr_error_loc_mac <= 1'b0;
end

// 以太网帧无效
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_header_error <= 1'b0;
	else if(icmpr_header_cnt == 4'd12)
		icmpr_header_error <= icmpr_error_loc_mac;
	else
		icmpr_header_error <= 1'b0;
end

// 以太网帧头14字节提取
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0) begin
		icmpr_read_loc_mac <= 32'd0;
		icmpr_read_dst_mac <= 48'd0;
		icmpr_header_type  <= 2'd0;
	end
	else if(icmpr_state == STATE_HEADER) begin
		case(icmpr_header_cnt)
			4'd0:  icmpr_read_loc_mac[47:40] <= icmpr_read_data;
			4'd1:  icmpr_read_loc_mac[39:32] <= icmpr_read_data;
			4'd2:  icmpr_read_loc_mac[31:24] <= icmpr_read_data;
			4'd3:  icmpr_read_loc_mac[23:16] <= icmpr_read_data;
			4'd4:  icmpr_read_loc_mac[15: 8] <= icmpr_read_data;
			4'd5:  icmpr_read_loc_mac[ 7: 0] <= icmpr_read_data;
			4'd6:  icmpr_read_dst_mac[47:40] <= icmpr_read_data;
			4'd7:  icmpr_read_dst_mac[39:32] <= icmpr_read_data;
			4'd8:  icmpr_read_dst_mac[31:24] <= icmpr_read_data;
			4'd9:  icmpr_read_dst_mac[23:16] <= icmpr_read_data;
			4'd10: icmpr_read_dst_mac[15: 8] <= icmpr_read_data;
			4'd11: icmpr_read_dst_mac[ 7: 0] <= icmpr_read_data;
			4'd12: icmpr_header_type [15: 8] <= icmpr_read_data;
			6'd13: icmpr_header_type [ 7: 0] <= icmpr_read_data;
		endcase
	end
end

// 以太网icmp包大小
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_datapack_len <= 8'd0;
	else if(icmpr_state == STATE_DATAPACK)
		icmpr_datapack_len <= icmpr_datapack_len - 1'b1;
	else
		icmpr_datapack_len <= icmpr_read_len - 8'd14;
end

// 以太网icmp包计数
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_datapack_rcnt <= 8'd0;
	else if(icmpr_state == STATE_DATAPACK)
		icmpr_datapack_rcnt <= icmpr_datapack_rcnt + 1'b1;
	else 
		icmpr_datapack_rcnt <= 8'd0;
end

// 提前预测以太网icmp包 IP头20+ICMP 头8个计数完成
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_datapack_done <= 1'b0;
	else if(icmpr_datapack_len == 8'd2)
		icmpr_datapack_done <= 1'b1;
	else
		icmpr_datapack_done <= 1'b0;
end

// 以太网icmp包异常
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_datapack_error <= 1'b0;
	else if(icmpr_state == STATE_DATAPACK)
		icmpr_datapack_error <= icmpr_header_type != PROTOCOL_ICMP ? 1'b1 : 1'b0;
	else
		icmpr_datapack_error <= 1'b0;
end

// 以太网数据包 IP包头20字节 参数
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0) begin
		icmpr_datapack_vr <= 4'd0;
		icmpr_datapack_fl <= 4'd0;
		icmpr_datapack_st <= 8'd0;
		icmpr_datapack_tl <= 16'd0;
		icmpr_datapack_if <= 16'd0;
		icmpr_datapack_lg <= 3'd0;
		icmpr_datapack_of <= 13'd0;
		icmpr_datapack_tt <= 8'd0;
		icmpr_datapack_pl <= 8'd0;
		icmpr_datapack_fc <= 16'd0;
		icmpr_read_dst_ip <= 32'd0;
		icmpr_read_loc_ip <= 32'd0;
	end
	else if(icmpr_state == STATE_IDLE)
		icmpr_read_loc_ip <= 32'd0;
	else if(icmpr_state == STATE_DATAPACK) begin
		case(icmpr_datapack_rcnt)
			5'd0: begin 
					   icmpr_datapack_vr[ 3: 0] <= icmpr_read_data[3:0]; 	// IP包头版本
						 icmpr_datapack_fl[ 3: 0] <= icmpr_read_data[7:4];	// IP包头首部长度
						end
			5'd1:  icmpr_datapack_st[ 7: 0] <= icmpr_read_data;  		  // IP包头服务类型
			5'd2:  icmpr_datapack_tl[15: 8] <= icmpr_read_data;	 		  // IP包头总长度
			5'd3:  icmpr_datapack_tl[ 7: 0] <= icmpr_read_data;	 		  // IP包头总长度
			5'd4:  icmpr_datapack_if[15: 8] <= icmpr_read_data;  		  // IP包头标识
			5'd5:  icmpr_datapack_if[ 7: 0] <= icmpr_read_data;  		  // IP包头标识
			5'd6: begin 
					   icmpr_datapack_lg[ 2: 0] <= icmpr_read_data[7:5]; 	// IP包头标志
						 icmpr_datapack_of[12: 8] <= icmpr_read_data[4:0];	// IP包头片偏移
						end  																								
			5'd7:  icmpr_datapack_of[ 7: 0] <= icmpr_read_data;  			// IP包头片偏移
			5'd8:  icmpr_datapack_tt[ 7: 0] <= icmpr_read_data;   		// IP包头TTL
			5'd9:  icmpr_datapack_pl[ 7: 0] <= icmpr_read_data;   		// IP包头协议
			5'd10: icmpr_datapack_fc[15: 8] <= icmpr_read_data;	  		// IP包头首部校验和
			5'd11: icmpr_datapack_fc[ 7: 0] <= icmpr_read_data;   		// IP包头首部校验和
			5'd12: icmpr_read_dst_ip[31:24] <= icmpr_read_data;   		// IP包头源IP
			5'd13: icmpr_read_dst_ip[23:16] <= icmpr_read_data;   		// IP包头源IP
			5'd14: icmpr_read_dst_ip[15: 8] <= icmpr_read_data;				// IP包头源IP 
			5'd15: icmpr_read_dst_ip[ 7: 0] <= icmpr_read_data;  			// IP包头源IP
			5'd16: icmpr_read_loc_ip[31:24] <= icmpr_read_data; 			// IP包头目的IP 
			5'd17: icmpr_read_loc_ip[23:16] <= icmpr_read_data;  			// IP包头目的IP 
			5'd18: icmpr_read_loc_ip[15: 8] <= icmpr_read_data;				// IP包头目的IP  
			5'd19: icmpr_read_loc_ip[ 7: 0] <= icmpr_read_data;  			// IP包头目的IP 
		endcase
	end
end

// 以太网IP包头目的IP错误
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_error_loc_ip <= 1'b0;
	else if(icmpr_state == STATE_IDLE)
		icmpr_error_loc_ip <= 1'b0;
	else if(icmpr_state == STATE_START)
		icmpr_error_loc_ip <= 1'b1;
	else if(icmpr_state == STATE_DATAPACK)
		icmpr_error_loc_ip <= icmpr_read_loc_ip == icmpr_loc_ip ? 1'b0 : 1'b1;
end

// 以太网IP包头协议错误
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_error_pl <= 1'b0;
	else if(icmpr_state == STATE_IDLE)
		icmpr_error_pl <= 1'b0;
	else if(icmpr_state == STATE_START)
		icmpr_error_pl <= 1'b1;
	else if(icmpr_state == STATE_DATAPACK)
		icmpr_error_pl <= icmpr_datapack_pl == 16'd1 ? 1'b0 : 1'b1;
end

// 以太网帧读取完成
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_read_done <= 1'b0;
	else if(icmpr_state == STATE_DONE)
		icmpr_read_done <= !icmpr_error_pl && !icmpr_error_loc_ip ? 1'b1 : 1'b0;
	else
		icmpr_read_done <= 1'b0;
end

// 输出以太网帧源IP
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_dst_ip <= 32'd0;
	else if(icmpr_state == STATE_DONE)
		icmpr_dst_ip <= icmpr_read_dst_ip;
end

// 输出以太网帧源MAC
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_dst_mac <= 48'd0;
	else if(icmpr_state == STATE_DONE)
		icmpr_dst_mac <= icmpr_read_dst_mac;
end

// icmp写计数
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_datapack_wcnt <= 5'd0;
	else if(icmpr_state == STATE_DATAPACK && icmpr_write_en == 1'b0)
		icmpr_datapack_wcnt <= icmpr_datapack_wcnt - 1'b1;
	else
		icmpr_datapack_wcnt <= 5'd24;
end

// icmp写bram 使能
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_write_en <= 1'b0;
	else if(icmpr_state == STATE_IDLE)
		icmpr_write_en <= 1'b0;
	else if(icmpr_datapack_wcnt == 5'd0)
		icmpr_write_en <= 1'b1;
	else if(icmpr_state == STATE_DONE)
		icmpr_write_en <= 1'b0;
end

// icmp写使能打拍
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_write_en_d1 <= 1'b0;
	else
		icmpr_write_en_d1 <= icmpr_write_en;
end

// icmp返回参数写入bram
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_write_data <= 8'd0;
	else 
		icmpr_write_data <= icmpr_read_data;
end

// icmp写地址计数
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_write_addr <= 8'd0;
	else if(icmpr_write_en)
		icmpr_write_addr <= icmpr_write_addr + 1'b1;
	else
		icmpr_write_addr <= 8'd0;
end

// icmp写长度
always @(posedge icmpr_clk or negedge icmpr_rst) begin
	if(icmpr_rst == 1'b0)
		icmpr_write_len <= 8'd0;
	else if(icmpr_write_en_d1 && !icmpr_write_en)
		icmpr_write_len <= icmpr_write_addr;
end
//  
endmodule
