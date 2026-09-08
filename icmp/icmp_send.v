//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 icmp_send
// 【功能描述】 以太网ICMP发送模块（封包）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.HEADER状态读取以太网帧头14字节固定长度。
// 【说明】 3.DATAPACK状态包结构有IP包头20字节固定和ICMP包N字节
// 【说明】 4.采用边读取bram封包写入bram，边计算累加和。
// 【说明】 5.为了精简高效IP包和ICMP包累加和分开独立计算的。
// 【说明】 6.ICMP包累加和计算最麻烦，采用先计算在写入bram。
// 【说明】 7.STATE_CHECK状态负责校验和回填，地址掉头。
// 【说明】 8.STATE_DONE状态负责告诉后端分包完成可以发送。
//
//////////////////////////////////////////////////////////////////////////////////
module icmp_send(
	input						 	icmpt_clk,					 // icmp时钟 
	input 		      	icmpt_rst,					 // icmp复位
	input 		 [31:0] icmpt_loc_ip,				 // icmp本地IP
	input 		 [47:0] icmpt_loc_mac,			 // icmp本地mac
	input 		 [31:0] icmpt_dst_ip,			 	 // icmp目标IP
	input 		 [47:0] icmpt_dst_mac,			 // icmp目标mac
	input      [ 7:0] icmpt_read_data,	 	 // icmp读数据
	input      [ 7:0] icmpt_read_len,		 	 // icmp读长度
	output reg [ 7:0] icmpt_read_addr,	 	 // icmp读地址
	input             icmpt_write_start,	 // icmp写启动
	output reg [ 7:0] icmpt_write_data,	 	 // icmp写数据
	output reg [ 7:0] icmpt_write_len,		 // icmp写长度
	output reg [ 7:0] icmpt_write_addr,	 	 // icmp写地址
	output reg   			icmpt_write_en,		 	 // icmp写使能
	output reg   			icmpt_write_done 	 	 // icmp写完成
);

// icmp 状态机定义  
localparam STATE_IDLE  		= 6'b000001;
localparam STATE_HEADER  	= 6'b000010;
localparam STATE_IPPACK  	= 6'b000100;
localparam STATE_DATAPACK = 6'b001000;
localparam STATE_CHECK    = 6'b010000;
localparam STATE_DONE     = 6'b100000;

// 以太网icmp参数
localparam ICMP_PROTOCOL_FT = 16'h0800;  // IP包协议帧类型

// SDRAM主控制器参数
reg [ 5:0] icmpt_state;
reg [ 5:0] icmpt_state_next;
reg        icmpt_write_start_d1;
reg        icmpt_write_start_d2;
reg [31:0] icmpt_write_loc_ip;
reg [47:0] icmpt_write_loc_mac;
reg [31:0] icmpt_write_dst_ip;
reg [47:0] icmpt_write_dst_mac;
reg [ 3:0] icmpt_header_cnt;
reg [ 7:0] icmpt_header_data;
reg        icmpt_header_done;
reg        icmpt_header_done_d1;
reg [ 4:0] icmpt_ippack_cnt;
reg [ 4:0] icmpt_ippack_len;
reg [ 7:0] icmpt_ippack_data;
reg [15:0] icmpt_ippack_tl;
reg        icmpt_ippack_sum_en;
reg        icmpt_ippack_sum_init;
reg        icmpt_ippack_done;
reg        icmpt_ippack_done_d1;
reg        icmpt_ippack_done_d2;
reg        icmpt_ippack_done_d3;
reg [ 7:0] icmpt_datapack_cnt;
reg [ 7:0] icmpt_datapack_len;
reg [ 7:0] icmpt_datapack_tl;
reg [ 7:0] icmpt_datapack_data;
reg [15:0] icmpt_datapack_sum;
reg        icmpt_datapack_done;  
reg        icmpt_datapack_done_d1; 
reg [ 7:0] icmpt_flowpack_data;
reg [ 2:0] icmpt_flowpack_cnt;
reg        icmpt_flowpack_en;
reg        icmpt_flowpack_en_d1;
reg        icmpt_flowpack_init;
reg [ 7:0] icmpt_check_data;
reg [ 5:0] icmpt_check_addr;
reg [ 2:0] icmpt_check_cnt; 
reg        icmpt_check_done;
reg        icmpt_read_en;
reg [ 7:0] icmpt_read_data_d1;
reg [ 7:0] icmpt_read_data_d2;
reg [ 7:0] icmpt_read_data_d3;
reg        icmpt_read_en_d1;
reg        icmpt_read_en_d2;
reg 			 icmpt_write_en_d1;
wire[15:0] icmpt_ippack_buf;
wire[15:0] icmpt_flowpack_buf;

// 寄存器
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_state <= STATE_IDLE;
	else
		icmpt_state <= icmpt_state_next;
end

// 状态跳转
always @(*) begin
	case(icmpt_state)
		STATE_IDLE: begin
			if(icmpt_write_start_d1)
				icmpt_state_next = STATE_HEADER;
			else
				icmpt_state_next = STATE_IDLE;
		end
		STATE_HEADER: begin
			if(icmpt_header_done)
				icmpt_state_next = STATE_IPPACK;
			else
				icmpt_state_next = STATE_HEADER;
		end
		STATE_IPPACK: begin
			if(icmpt_ippack_done)
				icmpt_state_next = STATE_DATAPACK;
			else
				icmpt_state_next = STATE_IPPACK;
		end
		STATE_DATAPACK: begin
			if(icmpt_datapack_done)
				icmpt_state_next = STATE_CHECK;
			else	
				icmpt_state_next = STATE_DATAPACK;
		end
		STATE_CHECK: begin
			if(icmpt_check_done)
				icmpt_state_next = STATE_DONE;
			else	
				icmpt_state_next = STATE_CHECK;
		end
		STATE_DONE: begin
			icmpt_state_next = STATE_IDLE;
		end
		default: icmpt_state_next = STATE_IDLE;
	endcase
end

// 写启动打1拍
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) begin
		icmpt_write_start_d1 <= 1'b0;
		icmpt_write_start_d2 <= 1'b0;
	end
	else begin
		icmpt_write_start_d1 <= icmpt_write_start;
		icmpt_write_start_d2 <= icmpt_write_start_d1;
	end
end

// 锁存输入源MAC和IP
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) begin
		icmpt_write_dst_ip <= 32'd0;
		icmpt_write_dst_mac <= 48'd0;
	end
	else if(icmpt_write_start_d1)begin
		icmpt_write_dst_ip <= icmpt_dst_ip;
		icmpt_write_dst_mac <= icmpt_dst_mac;
	end
end

// 锁存输入本地MAC和IP
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) begin
		icmpt_write_loc_ip <= 32'd0;
		icmpt_write_loc_mac <= 48'd0;
	end
	else if(icmpt_write_start_d1)begin
		icmpt_write_loc_ip <= icmpt_loc_ip;
		icmpt_write_loc_mac <= icmpt_loc_mac;
	end
end

// 一级锁存IP头总长度，二级跨时钟域累加
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) 
		icmpt_ippack_tl <= 16'd0;
	else if(icmpt_write_start_d1) 
		icmpt_ippack_tl <= {8'b0, icmpt_read_len};
	else if(icmpt_write_start_d2) 
		icmpt_ippack_tl <= icmpt_ippack_tl + 8'd24;
end

// 一级锁存ICMP包返回参数长度，二级跨时钟累加
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) 
		icmpt_datapack_tl <= 8'd0;
	else if(icmpt_write_start_d1) 
		icmpt_datapack_tl <= icmpt_read_len;
	else if(icmpt_write_start_d2) 
		icmpt_datapack_tl <= icmpt_datapack_tl + 8'd4;
end

// 以太网帧头计数(0-13)
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_header_cnt <= 4'd0;
	else if(icmpt_state == STATE_HEADER)
		icmpt_header_cnt <= icmpt_header_cnt + 1'b1;
	else
		icmpt_header_cnt <= 4'd0;
end

// 以太网帧头14字节数据
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_header_data <= 8'd0;
	else begin
		case(icmpt_header_cnt)
			4'd0:  icmpt_header_data <= icmpt_write_dst_mac[47:40]; 		 // 目的MAC地址
			4'd1:  icmpt_header_data <= icmpt_write_dst_mac[39:32]; 		 // 目的MAC地址
			4'd2:  icmpt_header_data <= icmpt_write_dst_mac[31:24]; 		 // 目的MAC地址
			4'd3:  icmpt_header_data <= icmpt_write_dst_mac[23:16]; 		 // 目的MAC地址
			4'd4:  icmpt_header_data <= icmpt_write_dst_mac[15: 8]; 		 // 目的MAC地址
			4'd5:  icmpt_header_data <= icmpt_write_dst_mac[ 7: 0]; 		 // 目的MAC地址
			4'd6:  icmpt_header_data <= icmpt_write_loc_mac[47:40]; 		 // 源MAC地址
			4'd7:  icmpt_header_data <= icmpt_write_loc_mac[39:32]; 		 // 源MAC地址
			4'd8:  icmpt_header_data <= icmpt_write_loc_mac[31:24]; 		 // 源MAC地址
			4'd9:  icmpt_header_data <= icmpt_write_loc_mac[23:16]; 		 // 源MAC地址
			4'd10: icmpt_header_data <= icmpt_write_loc_mac[15: 8]; 		 // 源MAC地址
			4'd11: icmpt_header_data <= icmpt_write_loc_mac[ 7: 0]; 		 // 源MAC地址
			4'd12: icmpt_header_data <= ICMP_PROTOCOL_FT[15:8];		 			 // 协议类型
			4'd13: icmpt_header_data <= ICMP_PROTOCOL_FT[ 7:0];		 			 // 协议类型
			default: icmpt_header_data <= 8'd0;
		endcase
	end
end

// 以太网帧头计数完成
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_header_done <= 1'b0;
	else if(icmpt_header_cnt == 4'd13)
		icmpt_header_done <= 1'b1;
	else
		icmpt_header_done <= 1'b0;
end

// 以太网帧IP包头计数
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_ippack_cnt <= 5'd0;
	else if(icmpt_state == STATE_IPPACK || icmpt_header_done)
		icmpt_ippack_cnt <= icmpt_ippack_cnt + 1'b1;
	else
		icmpt_ippack_cnt <= 5'd0;
end

// 以太网帧IP包数据20字节填充
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_ippack_data <= 8'd0;
	else begin
		case(icmpt_ippack_cnt)
			5'd0:  icmpt_ippack_data <= 8'h45; 		 										// IP包头版本+帧头
			5'd1:  icmpt_ippack_data <= 8'h00; 		 										// IP包头服务类型
			5'd2:  icmpt_ippack_data <= icmpt_ippack_tl[15:8]; 				// IP包头总长度
			5'd3:  icmpt_ippack_data <= icmpt_ippack_tl[ 7:0]; 				// IP包头总长度
			5'd4:  icmpt_ippack_data <= 8'h00;   	 										// IP包头标识
			5'd5:  icmpt_ippack_data <= 8'h00;	 	 										// IP包头标识
			5'd6:  icmpt_ippack_data <= 8'h00; 		 										// IP包头标志+偏移
			5'd7:  icmpt_ippack_data <= 8'h00; 		 										// IP包头标志+偏移
			5'd8:  icmpt_ippack_data <= 8'h80; 		 										// IP包头TTL
			5'd9:  icmpt_ippack_data <= 8'h01;  	 										// IP包头协议
			5'd10: icmpt_ippack_data <= 8'h00;		 	 									// IP包头校验
			5'd11: icmpt_ippack_data <= 8'h00; 	 		 									// IP包头校验
			5'd12: icmpt_ippack_data <= icmpt_write_loc_ip[31:24];	 	// IP包头源IP地址
			5'd13: icmpt_ippack_data <= icmpt_write_loc_ip[23:16];	 	// IP包头源IP地址
			5'd14: icmpt_ippack_data <= icmpt_write_loc_ip[15: 8];	 	// IP包头源IP地址
			5'd15: icmpt_ippack_data <= icmpt_write_loc_ip[ 7: 0];	 	// IP包头源IP地址
			5'd16: icmpt_ippack_data <= icmpt_write_dst_ip[31:24];	 	// IP包头目的IP地址
			5'd17: icmpt_ippack_data <= icmpt_write_dst_ip[23:16];	 	// IP包头目的IP地址
			5'd18: icmpt_ippack_data <= icmpt_write_dst_ip[15: 8];	 	// IP包头目的IP地址
			5'd19: icmpt_ippack_data <= icmpt_write_dst_ip[ 7: 0];	 	// IP包头目的IP地址
			default: icmpt_ippack_data <= 8'h00;					  
		endcase
	end
end

// 以太网帧IP包头倒计时
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_ippack_len <= 5'd0;
	else if(icmpt_state == STATE_IPPACK)
		icmpt_ippack_len <= icmpt_ippack_len - 1'b1;
	else
		icmpt_ippack_len <= 5'd20;
end

// 以太网帧IP包头计算完成
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_ippack_done <= 1'b0;
	else if(icmpt_ippack_len == 5'd2)
		icmpt_ippack_done <= 1'b1;
	else
		icmpt_ippack_done <= 1'b0;
end

// IP包完成延迟拍加载校验
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) begin
		icmpt_ippack_done_d1 <= 1'b0;
		icmpt_ippack_done_d2 <= 1'b0;
		icmpt_ippack_done_d3 <= 1'b0;
	end
	else begin
		icmpt_ippack_done_d1 <= icmpt_ippack_done;
		icmpt_ippack_done_d2 <= icmpt_ippack_done_d1;
		icmpt_ippack_done_d3 <= icmpt_ippack_done_d2;
	end
end

// 以太网帧IP包头校验使能
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_ippack_sum_en <= 1'b0;
	else if(icmpt_header_done)
		icmpt_ippack_sum_en <= 1'b1;
	else if(icmpt_ippack_done_d3)
		icmpt_ippack_sum_en <= 1'b0;
end

// 以太网帧IP包头校验初始化
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) 
		icmpt_ippack_sum_init <= 1'b0;
	else if(icmpt_state == STATE_IDLE)
		icmpt_ippack_sum_init <= 1'b1;
	else 
		icmpt_ippack_sum_init <= 1'b0;
end


// 以太网帧IP包头+ICMP包头+数据计数
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_datapack_cnt <= 8'd0;
	else if(icmpt_state == STATE_DATAPACK || icmpt_ippack_done)
		icmpt_datapack_cnt <= icmpt_datapack_cnt + 1'b1;
	else
		icmpt_datapack_cnt <= 8'd0;
end

// 以太网帧IP包长度倒计时
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_datapack_len <= 8'd0;
	else if(icmpt_state == STATE_DATAPACK)
		icmpt_datapack_len <= icmpt_datapack_len - 1'b1;
	else
		icmpt_datapack_len <= icmpt_datapack_tl;
end

// icmp读bram使能
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_read_en <= 1'b0;
	else if(icmpt_state == STATE_IDLE)
		icmpt_read_en <= 1'b0;
	else if(icmpt_ippack_len == 5'd3)
		icmpt_read_en <= 1'b1;
	else if(icmpt_datapack_len == 8'd5)
		icmpt_read_en <= 1'b0;
end

// icmp读取发送参数
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_read_addr <= 8'd0;
	else if(icmpt_read_en)
		icmpt_read_addr <= icmpt_read_addr + 1'b1;
	else
		icmpt_read_addr <= 8'd0;
end

// icmp读取发送参数打2拍提前输出
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) begin
		icmpt_read_data_d1 <= 8'd0;
		icmpt_read_data_d2 <= 8'd0;
		icmpt_read_data_d3 <= 8'd0;
	end
	else begin
		icmpt_read_data_d1 <= icmpt_read_data;
		icmpt_read_data_d2 <= icmpt_read_data_d1;
		icmpt_read_data_d3 <= icmpt_read_data_d2;
	end
end

// 以太网帧IP包头20字节+ICMP数据
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_datapack_data <= 8'd0;
	else begin
		case(icmpt_datapack_cnt)
			8'd0: icmpt_datapack_data <= 8'h00;		// ICMP类型
			8'd1: icmpt_datapack_data <= 8'h00;   // ICMP代码
			8'd2: icmpt_datapack_data <= 8'h00;   // ICMP校验
			8'd3: icmpt_datapack_data <= 8'h00;		// ICMP校验
			default: icmpt_datapack_data <= icmpt_read_data_d3;					
		endcase
	end
end

// 以太网帧icmp包计数完成
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_datapack_done <= 1'b0;
	else if(icmpt_datapack_len == 8'd2)
		icmpt_datapack_done <= 1'b1;
	else
		icmpt_datapack_done <= 1'b0;
end

// 以太网帧icmp包计数完成打拍
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) 
		icmpt_datapack_done_d1 <= 1'b0;
	else 
		icmpt_datapack_done_d1 <= icmpt_datapack_done;
end

// icmp包协议计数
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_flowpack_cnt <= 3'd0;
	else if(icmpt_flowpack_en) begin
		if(icmpt_flowpack_cnt < 5'd4)
			icmpt_flowpack_cnt <= icmpt_flowpack_cnt + 1'b1;
	end
	else
		icmpt_flowpack_cnt <= 3'd0;
end

// icmp协议数据校验
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_flowpack_data <= 8'd0;
	else begin
		case(icmpt_flowpack_cnt)
			3'd0: icmpt_flowpack_data <= 8'h00;				// ICMP类型
			3'd1: icmpt_flowpack_data <= 8'h00;   		// ICMP代码
			3'd2: icmpt_flowpack_data <= 8'h00;   		// ICMP校验
			3'd3: icmpt_flowpack_data <= 8'h00;				// ICMP校验		
			default: icmpt_flowpack_data <= icmpt_read_en ? icmpt_read_data : 8'd0;	// ICMP发送数据
		endcase
	end
end

// udp伪协议校验使能
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_flowpack_en <= 1'b0;
	else if(icmpt_ippack_len == 5'd5)
		icmpt_flowpack_en <= 1'b1;
	else if(icmpt_datapack_done)
		icmpt_flowpack_en <= 1'b0;
end

// icmp校验初始化
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_flowpack_init <= 1'b0;
	else if(icmpt_state == STATE_IDLE)
		icmpt_flowpack_init <= 1'b1;
	else 
		icmpt_flowpack_init <= 1'b0;
end

// icmp校验使能延迟1拍
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_flowpack_en_d1 <= 1'b0;
	else 
		icmpt_flowpack_en_d1 <= icmpt_flowpack_en;
end

// 以太网帧ICMP校验计数
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_check_cnt <= 3'd0;
	else if(icmpt_state == STATE_CHECK || icmpt_datapack_done)
		icmpt_check_cnt <= icmpt_check_cnt + 1'b1;
	else
		icmpt_check_cnt <= 3'd0;
end

// 以太网IP包头+UDP包校验
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_check_data <= 8'd0;
	else begin
		case(icmpt_check_cnt)
			3'd0: icmpt_check_data <= icmpt_ippack_buf[15:8]; 	// IP包头校验
			3'd1: icmpt_check_data <= icmpt_ippack_buf[ 7:0]; 	// IP包头校验
			3'd2: icmpt_check_data <= icmpt_flowpack_buf[15:8]; // ICMP包头校验
			3'd3: icmpt_check_data <= icmpt_flowpack_buf[ 7:0]; // ICMP包头校验
			default: icmpt_check_data <= 8'd0;
		endcase
	end
end

// 以太网IP包头+UDP包地址回退
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_check_addr <= 6'd0;
	else begin
		case(icmpt_check_cnt)
			3'd0: icmpt_check_addr <= 6'h18; 	// IP包头校验地址
			3'd1: icmpt_check_addr <= 6'h19; 	// IP包头校验地址
			3'd2: icmpt_check_addr <= 6'h24; 	// ICMP包头校验地址
			3'd3: icmpt_check_addr <= 6'h25; 	// ICMP包头校验地址
			default: icmpt_check_addr <= 6'd0;
		endcase
	end
end

// 以太网帧ICMP校验计数完成
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_check_done <= 1'b0;
	else if(icmpt_check_cnt == 3'd3)
		icmpt_check_done <= 1'b1;
	else
		icmpt_check_done <= 1'b0;
end

// BRAM写使能
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_write_en_d1 <= 1'b0;
	else if(icmpt_state == STATE_HEADER)
		icmpt_write_en_d1 <= 1'b1;
	else if(icmpt_check_done)
		icmpt_write_en_d1 <= 1'b0;
end

// BRAM写使能打1拍
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) 
		icmpt_write_en <= 1'b0;
	else
		icmpt_write_en    <= icmpt_write_en_d1;
end

// bram写地址和数据延迟1拍计数
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0) 
		icmpt_write_addr <= 8'd0;
	else if(icmpt_state == STATE_CHECK)
		icmpt_write_addr <= {2'b0, icmpt_check_addr};
	else if(icmpt_write_en)
		 icmpt_write_addr <= icmpt_write_addr + 1'b1;
	else
		icmpt_write_addr <= 8'd0;	
end

// ICMP以太网帧写入BRAM
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_write_data <= 8'd0;
	else begin
		case(icmpt_state)
			STATE_HEADER: 	icmpt_write_data <= icmpt_header_data;
			STATE_IPPACK:   icmpt_write_data <= icmpt_ippack_data;
			STATE_DATAPACK: icmpt_write_data <= icmpt_datapack_data;
			STATE_CHECK:    icmpt_write_data <= icmpt_check_data;
			default:        icmpt_write_data <= 8'h00;
		endcase
	end
end

// 提前预判icmp发送包写入bram完成填补校验延迟
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_write_done <= 1'b0;
	else if(icmpt_datapack_done_d1)
		icmpt_write_done <= 1'b1;
	else
		icmpt_write_done <= 1'b0;
end

// 锁存icmp发送包写入长度
always @(posedge icmpt_clk or negedge icmpt_rst) begin
	if(icmpt_rst == 1'b0)
		icmpt_write_len <= 6'd0;
	else if(icmpt_datapack_done_d1)
		icmpt_write_len <= icmpt_write_addr + 8'd1;
end

// IP包20字节校验计算
eth_check_sum icmp_ippack_sum( 
	.check_clk(icmpt_clk),									// sum校验时钟 
	.check_rst(icmpt_rst),									// sum校验复位
	.check_sum_in(icmpt_ippack_data),				// sum校验输入
	.check_sum_en(icmpt_ippack_sum_en),			// sum校验使能
	.check_sum_val(icmpt_ippack_sum_init), 	// sum校验初始化 
	.check_sum_out(icmpt_ippack_buf)     		// sum校验输出
);

// IP包20字节校验计算
eth_check_sum icmp_datapack_sum( 
	.check_clk(icmpt_clk),									// sum校验时钟 
	.check_rst(icmpt_rst),									// sum校验复位
	.check_sum_in(icmpt_flowpack_data),	  	// sum校验输入
	.check_sum_en(icmpt_flowpack_en_d1),	  // sum校验使能
	.check_sum_val(icmpt_flowpack_init),		// sum校验初始化 
	.check_sum_out(icmpt_flowpack_buf)     	// sum校验输出
);

endmodule
