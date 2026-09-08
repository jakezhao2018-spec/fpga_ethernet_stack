//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 arp_send
// 【功能描述】 以太网ARP发送模块（封包）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.HEADER状态封包以太网帧头14字节固定长度。
// 【说明】 3.DATAPACK状态封包ARP协议包28字节固定。
// 【说明】 4.arp只之前应答模式，不支持请求模式
// 【说明】 5.STATE_DONE状态负责告诉后端分包完成可以发送
// 【说明】 6.
//
//////////////////////////////////////////////////////////////////////////////////
module arp_send(
	input						 	arpt_clk,					 // arp时钟 
	input 		      	arpt_rst,					 // arp复位
	input 		 [31:0] arpt_loc_ip,			 // arp本地IP
	input 		 [47:0] arpt_loc_mac,			 // arp本地mac
	input 		 [31:0] arpt_dst_ip,			 // arp目标IP
	input 		 [47:0] arpt_dst_mac,			 // arp目标mac
	input             arpt_write_start,	 // arp写启动
	output reg [ 7:0] arpt_write_data,	 // arp写数据
	output reg [ 5:0] arpt_write_len,		 // arp写长度
	output reg [ 5:0] arpt_write_addr,	 // arp写地址
	output reg   			arpt_write_en,		 // arp写使能
	output reg   			arpt_write_done 	 // arp写完成
);

// arp 状态机定义  
localparam STATE_IDLE  		= 5'b00001;
localparam STATE_START    = 5'b00010;
localparam STATE_HEADER  	= 5'b00100;
localparam STATE_DATAPACK = 5'b01000;
localparam STATE_DONE     = 5'b10000;

// 以太网arp参数
localparam ARP_PROTOCOL_FT = 16'h0806;   // arp协议帧类型
localparam ARP_DATAPACK_HT = 16'h0001;	 // arp包硬件类型
localparam ARP_DATAPACK_PT = 16'h0800;	 // arp包协议类型
localparam ARP_DATAPACK_OP = 16'h0002;	 // arp包请求类型1请求2应答
localparam ARP_DATAPACK_HL = 8'h06;			 // arp包协议地址长度
localparam ARP_DATAPACK_PL = 8'h04;			 // arp包硬件地址长度

// SDRAM主控制器参数
reg [ 4:0] arpt_state;
reg [ 4:0] arpt_state_next;
reg [31:0] arpt_write_loc_ip;
reg [47:0] arpt_write_loc_mac;
reg [31:0] arpt_write_dst_ip;
reg [47:0] arpt_write_dst_mac;
reg        arpt_write_start_d1;
reg        arpt_write_en_d1;
reg [ 3:0] arpt_header_cnt;
reg [ 7:0] arpt_header_data;
reg        arpt_header_done;
reg [ 4:0] arpt_datapack_cnt;
reg [ 7:0] arpt_datapack_data;
reg        arpt_datapack_done;

// 寄存器
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_state <= STATE_IDLE;
	else
		arpt_state <= arpt_state_next;
end

// 状态跳转
always @(*) begin
	case(arpt_state)
		STATE_IDLE: begin
			if(arpt_write_start_d1)
				arpt_state_next = STATE_START;
			else
				arpt_state_next = STATE_IDLE;
		end
		STATE_START: begin
			arpt_state_next = STATE_HEADER;
		end
		STATE_HEADER: begin
			if(arpt_header_done)
				arpt_state_next = STATE_DATAPACK;
			else
				arpt_state_next = STATE_HEADER;
		end
		STATE_DATAPACK: begin
			if(arpt_datapack_done)
				arpt_state_next = STATE_DONE;
			else	
				arpt_state_next = STATE_DATAPACK;
		end
		STATE_DONE: begin
			arpt_state_next = STATE_IDLE;
		end
		default: arpt_state_next = STATE_IDLE;
	endcase
end

// 写启动打1拍
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_start_d1 <= 1'b0;
	else 
		arpt_write_start_d1 <= arpt_write_start;
end

// 锁存输入源MAC和IP
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0) begin
		arpt_write_dst_ip <= 32'd0;
		arpt_write_dst_mac <= 48'd0;
	end
	else if(arpt_state == STATE_START)begin
		arpt_write_dst_ip <= arpt_dst_ip;
		arpt_write_dst_mac <= arpt_dst_mac;
	end
end

// 锁存输入本地MAC和IP
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0) begin
		arpt_write_loc_ip <= 32'd0;
		arpt_write_loc_mac <= 48'd0;
	end
	else if(arpt_state == STATE_START)begin
		arpt_write_loc_ip <= arpt_loc_ip;
		arpt_write_loc_mac <= arpt_loc_mac;
	end
end

// BRAM写使能延迟1拍
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_en_d1 <= 1'b0;
	else if(arpt_state == STATE_HEADER)
		arpt_write_en_d1 <= 1'b1;
	else if(arpt_datapack_done)
		arpt_write_en_d1 <= 1'b0;
end

// BRAM写使能打拍
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_en <= 1'b0;
	else
		arpt_write_en <= arpt_write_en_d1;
end

// BRAM写地址
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_addr <= 5'd0;
	else if(arpt_write_en)
		arpt_write_addr <= arpt_write_addr + 1'b1;
	else
		arpt_write_addr <= 5'd0;	
end

// 以太网帧头计数(0-13)
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_header_cnt <= 4'd0;
	else if(arpt_state == STATE_HEADER)
		arpt_header_cnt <= arpt_header_cnt + 1'b1;
	else
		arpt_header_cnt <= 4'd0;
end

// 以太网帧头数据
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_header_data <= 8'd0;
	else begin
		case(arpt_header_cnt)
			4'd0:  arpt_header_data <= arpt_write_dst_mac[47:40]; 		// 目的MAC地址
			4'd1:  arpt_header_data <= arpt_write_dst_mac[39:32]; 		// 目的MAC地址
			4'd2:  arpt_header_data <= arpt_write_dst_mac[31:24]; 		// 目的MAC地址
			4'd3:  arpt_header_data <= arpt_write_dst_mac[23:16]; 		// 目的MAC地址
			4'd4:  arpt_header_data <= arpt_write_dst_mac[15: 8]; 		// 目的MAC地址
			4'd5:  arpt_header_data <= arpt_write_dst_mac[ 7: 0]; 		// 目的MAC地址
			4'd6:  arpt_header_data <= arpt_write_loc_mac[47:40]; 		// 源MAC地址
			4'd7:  arpt_header_data <= arpt_write_loc_mac[39:32]; 		// 源MAC地址
			4'd8:  arpt_header_data <= arpt_write_loc_mac[31:24]; 		// 源MAC地址
			4'd9:  arpt_header_data <= arpt_write_loc_mac[23:16]; 		// 源MAC地址
			4'd10: arpt_header_data <= arpt_write_loc_mac[15: 8]; 		// 源MAC地址
			4'd11: arpt_header_data <= arpt_write_loc_mac[ 7: 0]; 		// 源MAC地址
			4'd12: arpt_header_data <= ARP_PROTOCOL_FT[15:8];					// 协议类型
			4'd13: arpt_header_data <= ARP_PROTOCOL_FT[ 7:0];					// 协议类型
			default: arpt_header_data <= 8'd0;
		endcase
	end
end

// 以太网帧头计数完成
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_header_done <= 1'b0;
	else if(arpt_state == STATE_HEADER)
		arpt_header_done <= arpt_header_cnt == 4'd13 ? 1'b1 : 1'b0;
	else
		arpt_header_done <= 1'b0;
end

// 以太网帧arp协议计数(0-27)
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_datapack_cnt <= 5'd0;
	else if(arpt_state == STATE_DATAPACK || arpt_header_done)
		arpt_datapack_cnt <= arpt_datapack_cnt + 1'b1;
	else
		arpt_datapack_cnt <= 5'd0;
end

// 以太网帧arp协议数据
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_datapack_data <= 8'd0;
	else begin
		case(arpt_datapack_cnt)
			5'd0:  arpt_datapack_data <= ARP_DATAPACK_HT[15:8]; 			// 硬件类型
			5'd1:  arpt_datapack_data <= ARP_DATAPACK_HT[ 7:0]; 			// 硬件类型
			5'd2:  arpt_datapack_data <= ARP_DATAPACK_PT[15:8]; 			// 协议类型
			5'd3:  arpt_datapack_data <= ARP_DATAPACK_PT[ 7:0]; 			// 协议类型
			5'd4:  arpt_datapack_data <= ARP_DATAPACK_HL;   					// 硬件地址长度
			5'd5:  arpt_datapack_data <= ARP_DATAPACK_PL;	 		 				// 协议地址长度
			5'd6:  arpt_datapack_data <= ARP_DATAPACK_OP[15:8]; 			// OP,01请求02应答
			5'd7:  arpt_datapack_data <= ARP_DATAPACK_OP[ 7:0]; 			// OP,01请求02应答
			5'd8:  arpt_datapack_data <= arpt_write_loc_mac[47:40];  	// 发送端MAC地址
			5'd9:  arpt_datapack_data <= arpt_write_loc_mac[39:32];  	// 发送端MAC地址
			5'd10: arpt_datapack_data <= arpt_write_loc_mac[31:24];  	// 发送端MAC地址
			5'd11: arpt_datapack_data <= arpt_write_loc_mac[23:16];  	// 发送端MAC地址
			5'd12: arpt_datapack_data <= arpt_write_loc_mac[15: 8];  	// 发送端MAC地址
			5'd13: arpt_datapack_data <= arpt_write_loc_mac[ 7: 0];  	// 发送端MAC地址
			5'd14: arpt_datapack_data <= arpt_write_loc_ip[31:24];	 	// 发送端IP地址
			5'd15: arpt_datapack_data <= arpt_write_loc_ip[23:16];	 	// 发送端IP地址
			5'd16: arpt_datapack_data <= arpt_write_loc_ip[15: 8];	 	// 发送端IP地址
			5'd17: arpt_datapack_data <= arpt_write_loc_ip[ 7: 0];	 	// 发送端IP地址
			5'd18: arpt_datapack_data <= arpt_write_dst_mac[47:40];  	// 目标MAC地址
			5'd19: arpt_datapack_data <= arpt_write_dst_mac[39:32];  	// 目标MAC地址
			5'd20: arpt_datapack_data <= arpt_write_dst_mac[31:24];  	// 目标MAC地址
			5'd21: arpt_datapack_data <= arpt_write_dst_mac[23:16];  	// 目标MAC地址
			5'd22: arpt_datapack_data <= arpt_write_dst_mac[15: 8];  	// 目标MAC地址
			5'd23: arpt_datapack_data <= arpt_write_dst_mac[ 7: 0];  	// 目标MAC地址
			5'd24: arpt_datapack_data <= arpt_write_dst_ip[31:24];	 	// 目标IP地址
			5'd25: arpt_datapack_data <= arpt_write_dst_ip[23:16];	 	// 目标IP地址
			5'd26: arpt_datapack_data <= arpt_write_dst_ip[15: 8];	 	// 目标IP地址
			5'd27: arpt_datapack_data <= arpt_write_dst_ip[ 7: 0];	 	// 目标IP地址
			default: arpt_datapack_data <= 8'd0;
		endcase
	end
end

// 以太网帧arp协议计数完成
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_datapack_done <= 1'b0;
	else if(arpt_datapack_cnt == 5'd27)
		arpt_datapack_done <= 1'b1;
	else
		arpt_datapack_done <= 1'b0;
end

// arp以太网帧写入BRAM
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_data <= 8'd0;
	else begin
		case(arpt_state)
			STATE_HEADER: 	arpt_write_data <= arpt_header_data;
			STATE_DATAPACK: arpt_write_data <= arpt_datapack_data;
			default:        arpt_write_data <= 8'h00;
		endcase
	end
end

// arp发送包写入bram完成
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_done <= 1'b0;
	else if(arpt_state == STATE_DONE)
		arpt_write_done <= 1'b1;
	else
		arpt_write_done <= 1'b0;
end

// arp发送包写入长度
always @(posedge arpt_clk or negedge arpt_rst) begin
	if(arpt_rst == 1'b0)
		arpt_write_len <= 6'd0;
	else if(arpt_state == STATE_DONE)
		arpt_write_len <= arpt_write_addr;
end

endmodule
