//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 arp_receive
// 【功能描述】 以太网arp接收模块（输出接收IP、MAC）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.START状态读取arp协议bram缓存，为了解决读潜伏期2拍
// 【说明】 3.HEADER状态读取以太网帧头14字节固定长度。
// 【说明】 4.DATAPACK包结构ICMP固定28个字节
// 【说明】 5.STATE_DONE告诉后端发送模块，可以发送应答包。
// 【说明】 6.
//
//////////////////////////////////////////////////////////////////////////////////
module arp_receive(
	input				 			arpr_clk,					// arp时钟 
	input        			arpr_rst,					// arp复位
	input  		 [31:0] arpr_loc_ip,			// arp本地IP
	input  		 [47:0] arpr_loc_mac,			// arp本地mac
	output reg [31:0] arpr_dst_ip,			// arp目标IP
	output reg [47:0] arpr_dst_mac,			// arp目标mac
	input             arpr_read_start,	// arp读启动
	input  		 [ 7:0] arpr_read_data,		// arp读数据
	input      [10:0] arpr_read_len,		// arp读长度
	output reg [10:0] arpr_read_addr,		// arp读计数
	output reg   			arpr_read_done		// arp读完成
);

// arp 状态机定义  
localparam STATE_IDLE  		= 5'b00001;
localparam STATE_START    = 5'b00010;
localparam STATE_HEADER  	= 5'b00100;
localparam STATE_DATAPACK = 5'b01000;
localparam STATE_DONE 		= 5'b10000;

// arp 包参数
localparam PROTOCOL_ARP   = 16'h0806; 

// SDRAM主控制器参数
reg [ 4:0] arpr_state;
reg [ 4:0] arpr_state_next;
reg [31:0] arpr_read_loc_ip;
reg [47:0] arpr_read_loc_mac;
reg [31:0] arpr_read_dst_ip;
reg [47:0] arpr_read_dst_mac;
reg        arpr_read_en;
reg [ 2:0] arpr_start_cnt;
reg        arpr_start_done;
reg        arpr_header_done;
reg [ 3:0] arpr_header_cnt;
reg [15:0] arpr_header_type;
reg        arpr_header_error;
reg [ 4:0] arpr_datapack_cnt;
reg        arpr_datapack_done;
reg        arpr_datapack_error;
reg [15:0] arpr_datapack_ht;
reg [15:0] arpr_datapack_pt;
reg [15:0] arpr_datapack_op;
reg [ 7:0] arpr_datapack_hl;
reg [ 7:0] arpr_datapack_pl;
reg        arpr_error_op;
reg        arpr_error_loc_ip;

// 寄存器
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_state <= STATE_IDLE;
	else
		arpr_state <= arpr_state_next;
end

// 状态跳转
always @(*) begin
	case(arpr_state)
		STATE_IDLE: begin
			if(arpr_read_start)
				arpr_state_next = STATE_START;
			else
				arpr_state_next = STATE_IDLE;
		end
		STATE_START: begin
			if(arpr_start_done)
				arpr_state_next = STATE_HEADER;
			else
				arpr_state_next = STATE_START;
		end
		STATE_HEADER: begin
			if(arpr_header_error)
				arpr_state_next = STATE_IDLE;
			else if(arpr_header_done)
				arpr_state_next = STATE_DATAPACK;
			else
				arpr_state_next = STATE_HEADER;
		end
		STATE_DATAPACK: begin
			if(arpr_datapack_done)
				arpr_state_next = STATE_DONE;
			else if(arpr_datapack_error)
				arpr_state_next = STATE_IDLE;
			else
				arpr_state_next = STATE_DATAPACK;
		end
		STATE_DONE: begin
			arpr_state_next = STATE_IDLE;
		end
		default: arpr_state_next = STATE_IDLE;
	endcase
end

// arp读启动计数
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_start_cnt <= 3'd0;
	else if(arpr_state == STATE_START)
		arpr_start_cnt <= arpr_start_cnt + 1'b1;
	else
		arpr_start_cnt <= 3'd0;
end

// arp读启动完成
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_start_done <= 1'b0;
	else if(arpr_start_cnt == 3'd1)
		arpr_start_done <= 1'b1;
	else
		arpr_start_done <= 1'b0;
end

// 读数据使能
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_read_en <= 1'b0;
	else if(arpr_state == STATE_IDLE)
		arpr_read_en <= 1'b0;
	else if(arpr_state == STATE_START)
		arpr_read_en <= 1'b1;
end

// 读取BRAM地址计数
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_read_addr <= 11'd0;
	else if(arpr_read_en)
		arpr_read_addr <= arpr_read_addr + 1'b1;
	else
		arpr_read_addr <= 11'd0;
end

// 以太网帧头计数器
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_header_cnt <= 4'd0;
	else if(arpr_state == STATE_HEADER)
		arpr_header_cnt <= arpr_header_cnt + 1'b1;
	else
		arpr_header_cnt <= 4'd0;
end

// 提取预测以太网帧头读取完成
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_header_done <= 1'b0;
	else if(arpr_header_cnt == 4'd12)
		arpr_header_done <= 1'b1;
	else
		arpr_header_done <= 1'b0;
end

// 以太网帧无效
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_header_error <= 1'b0;
	else if(arpr_header_cnt == 4'd12) begin
		if(arpr_read_loc_mac == arpr_loc_mac) 
			arpr_header_error <= 1'b0;
		else if(arpr_read_loc_mac == 48'hff_ff_ff_ff_ff_ff)
			arpr_header_error <= 1'b0;
		else
			arpr_header_error <= 1'b1;
	end
	else
		arpr_header_error <= 1'b0;
end

// 以太网帧头14字节提取
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0) begin
		arpr_read_loc_mac <= 32'd0;
		arpr_read_dst_mac <= 48'd0;
		arpr_header_type  <= 2'd0;
	end
	else if(arpr_state == STATE_HEADER) begin
		case(arpr_header_cnt)
			4'd0:  arpr_read_loc_mac[47:40] <= arpr_read_data;
			4'd1:  arpr_read_loc_mac[39:32] <= arpr_read_data;
			4'd2:  arpr_read_loc_mac[31:24] <= arpr_read_data;
			4'd3:  arpr_read_loc_mac[23:16] <= arpr_read_data;
			4'd4:  arpr_read_loc_mac[15: 8] <= arpr_read_data;
			4'd5:  arpr_read_loc_mac[ 7: 0] <= arpr_read_data;
			4'd6:  arpr_read_dst_mac[47:40] <= arpr_read_data;
			4'd7:  arpr_read_dst_mac[39:32] <= arpr_read_data;
			4'd8:  arpr_read_dst_mac[31:24] <= arpr_read_data;
			4'd9:  arpr_read_dst_mac[23:16] <= arpr_read_data;
			4'd10: arpr_read_dst_mac[15: 8] <= arpr_read_data;
			4'd11: arpr_read_dst_mac[ 7: 0] <= arpr_read_data;
			4'd12: arpr_header_type [15: 8] <= arpr_read_data;
			6'd13: arpr_header_type [ 7: 0] <= arpr_read_data;
		endcase
	end
end

// 以太网arp包计数
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_datapack_cnt <= 5'd0;
	else if(arpr_state == STATE_DATAPACK)
		arpr_datapack_cnt <= arpr_datapack_cnt + 1'b1;
	else 
		arpr_datapack_cnt <= 5'd0;
end

// 提前预测以太网arp包28个字节计数完成
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_datapack_done <= 1'b0;
	else if(arpr_datapack_cnt == 5'd27)
		arpr_datapack_done <= 1'b1;
	else
		arpr_datapack_done <= 1'b0;
end

// 以太网arp包异常
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_datapack_error <= 1'b0;
	else if(arpr_state == STATE_DATAPACK)
		arpr_datapack_error <= arpr_header_type != PROTOCOL_ARP ? 1'b1 : 1'b0;
	else
		arpr_datapack_error <= 1'b0;
end

// 以太网arp包8个字节参数
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0) begin
		arpr_datapack_ht <= 16'd0;
		arpr_datapack_pt <= 16'd0;
		arpr_datapack_op <= 16'd0;
		arpr_datapack_hl <= 8'd0;
		arpr_datapack_pl <= 8'd0;
	end
	else if(arpr_state == STATE_IDLE)
		arpr_datapack_op <= 16'd0;
	else if(arpr_state == STATE_DATAPACK) begin
		case(arpr_datapack_cnt)
			5'd0: arpr_datapack_ht[15:8] <= arpr_read_data;  // arp硬件类型
			5'd1: arpr_datapack_ht[ 7:0] <= arpr_read_data;  // arp硬件类型
			5'd2: arpr_datapack_pt[15:8] <= arpr_read_data;	 // arp协议类型
			5'd3: arpr_datapack_pt[ 7:0] <= arpr_read_data;  // arp协议类型
			5'd4: arpr_datapack_hl[ 7:0] <= arpr_read_data;	 // arp硬件地址长度
			5'd5: arpr_datapack_pl[ 7:0] <= arpr_read_data;	 // arp协议地址长度
			5'd6: arpr_datapack_op[15:8] <= arpr_read_data;	 // arp操作码
			5'd7: arpr_datapack_op[ 7:0] <= arpr_read_data;  // arp操作码
		endcase
	end
end

// 提取以太网arp包源IP
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_read_dst_ip <= 32'd0;
	else if(arpr_state == STATE_DATAPACK) begin
		case(arpr_datapack_cnt)
			5'd14: arpr_read_dst_ip[31:24] <= arpr_read_data;  
			5'd15: arpr_read_dst_ip[23:16] <= arpr_read_data;  
			5'd16: arpr_read_dst_ip[15: 8] <= arpr_read_data;	 
			5'd17: arpr_read_dst_ip[ 7: 0] <= arpr_read_data;  
		endcase
	end
end

// 提取以太网arp包目的IP
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_read_loc_ip <= 32'd0;
	else if(arpr_state == STATE_IDLE)
		arpr_read_loc_ip <= 32'd0;
	else if(arpr_state == STATE_DATAPACK) begin
		case(arpr_datapack_cnt)
			5'd24: arpr_read_loc_ip[31:24] <= arpr_read_data;  
			5'd25: arpr_read_loc_ip[23:16] <= arpr_read_data;  
			5'd26: arpr_read_loc_ip[15: 8] <= arpr_read_data;	 
			5'd27: arpr_read_loc_ip[ 7: 0] <= arpr_read_data;  
		endcase
	end
end

// 以太网apr目的IP错误
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_error_loc_ip <= 1'b0;
	else if(arpr_state == STATE_IDLE)
		arpr_error_loc_ip <= 1'b0;
	else if(arpr_state == STATE_START)
		arpr_error_loc_ip <= 1'b1;
	else if(arpr_state == STATE_DATAPACK)
		arpr_error_loc_ip <= arpr_read_loc_ip == arpr_loc_ip ? 1'b0 : 1'b1;
end

// 以太网arp包op请求错误
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_error_op <= 1'b0;
	else if(arpr_state == STATE_IDLE)
		arpr_error_op <= 1'b0;
	else if(arpr_state == STATE_START)
		arpr_error_op <= 1'b1;
	else if(arpr_state == STATE_DATAPACK)
		arpr_error_op <= arpr_datapack_op == 16'd1 ? 1'b0 : 1'b1;
end

// 以太网帧读取完成
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_read_done <= 1'b0;
	else if(arpr_state == STATE_DONE)
		arpr_read_done <= !arpr_error_op && !arpr_error_loc_ip ? 1'b1 : 1'b0;
	else
		arpr_read_done <= 1'b0;
end

// 输出以太网帧源IP
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_dst_ip <= 32'd0;
	else if(arpr_state == STATE_DONE)
		arpr_dst_ip <= arpr_read_dst_ip;
end

// 输出以太网帧源MAC
always @(posedge arpr_clk or negedge arpr_rst) begin
	if(arpr_rst == 1'b0)
		arpr_dst_mac <= 48'd0;
	else if(arpr_state == STATE_DONE)
		arpr_dst_mac <= arpr_read_dst_mac;
end
//  
endmodule
