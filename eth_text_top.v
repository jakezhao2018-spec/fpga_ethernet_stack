//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_text_top
// 【功能描述】 以太网测试顶层模块
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.以太网接收时钟进入pll输出pll_125mhz和pll_125mhz90读
// 【说明】 2.把以太网接收到的数据写入到发送缓冲区。
// 【说明】 3.实现收到什么数据发送什么数据测试。
// 【说明】 4.
// 【说明】 5.
// 【说明】 6.
//
//////////////////////////////////////////////////////////////////////////////////
module eth_text_top(
	input			 	 	sys_clk,					// 系统时钟 
	input      	 	sys_rst,					// 系统复位
	input         eth_rxd_clk,			// 以太网接收时钟
	input  [3:0]  eth_rxd_data,			// 以太网接收数据
	input   		  eth_rxd_valid,		// 以太网接收有效
	output 			  eth_txd_clk,			// 以太网发送时钟
	output [3:0]  eth_txd_data,			// 以太网发送数据
	output			  eth_txd_en,				// 以太网发送使能
	output			  eth_phy_rst,			// 以太网MAC复位
	output        eth_phy_mdc,			// 以太网MAC时钟
	inout         eth_phy_mdio			// 以太网MAC数据
);

// loc本地，dst远程
localparam [31:0] SET_LOC_IP  = {8'd192, 8'd168, 8'd1, 8'd201};
localparam [47:0] SET_LOC_MAC = {8'h44, 8'hb7, 8'hd0, 8'hd5, 8'h25, 8'h7e};
localparam [15:0] SET_LOC_PORT= 16'd46002;

//
wire pll_125mhz;
wire pll_125mhz_90;
wire pll_locked_1;

// pll_2以太网收发时钟
alt_pll_1 eth_rxd_clk_inst(
	.areset(!sys_rst),      // 复位输入
	.inclk0(eth_rxd_clk),		// 输入125mhz时钟
	.c0(pll_125mhz_90),	 		// 输出125mhz_90度接收时钟
	.c1(pll_125mhz),	   		// 输出125mhz 发送时钟
	.locked(pll_locked_1)   // 初始使能
);

//
reg [22:0] sys_rst_cnt;
always @(posedge sys_clk or negedge sys_rst) begin
	if(sys_rst == 1'b0)
		sys_rst_cnt <= 23'd0;
	else if(sys_rst_cnt == 23'd5000000)
		sys_rst_cnt <= 23'd0;
	else 
		sys_rst_cnt <= sys_rst_cnt + 1'b1;
end

// 以太网写
wire       eth_write_clk;		 // 以太网写bram时钟
wire       eth_write_req;		 // 以太网写bram请求
wire       eth_write_ack;		 // 以太网写bram应答
wire [ 7:0]eth_write_data;	 // 以太网写bram数据
wire [ 8:0]eth_write_cnt;		 // 以太网写bram计数器
wire [ 8:0]eth_write_offset; // 以太网写bram偏移双缓冲乒乓
wire [ 8:0]eth_write_len;		 // 以太网写bram数据大小
wire       eth_write_syn;  	 // 以太网写sdram双缓冲乒乓同步
wire       eth_write_en;		 // 以太网写bram使能

// 以太网读
wire       eth_read_clk;		 // 以太网读时钟
wire       eth_read_req;		 // 以太网读请求(主动)
wire       eth_read_ack;     // 以太网读应答
wire       eth_read_syn;   	 // 以太网读sdram双缓冲乒乓同步
wire [ 7:0]eth_read_data;		 // 以太网读bram数据
wire [10:0]eth_read_cnt;     // 以太网读bram计数
reg  [10:0]eth_read_len;		 // 以太网读bram长度
reg  [10:0]eth_read_offset;	 // 以太网读bram偏移双缓冲乒乓

//
reg        eth_read_req_d1;
reg        eth_read_req_d2;
reg [8:0]  eth_read_req_d3;

// 写请求打拍
always @(posedge eth_read_clk or negedge sys_rst) begin 
	if(sys_rst == 1'b0) begin
		eth_read_req_d1 <= 1'b0;
		eth_read_req_d2 <= 1'b0;
	end
	else begin
		eth_read_req_d1 <= eth_read_req;
		eth_read_req_d2 <= eth_read_req_d1;
	end
end

// 延迟发送帧间隔,压力测试的时候用，实际项目中基本不需要考虑
always @(posedge eth_read_clk or negedge sys_rst) begin 
	if(sys_rst == 1'b0)
		eth_read_req_d3 <= 9'd0;
	else 
		eth_read_req_d3 <= {eth_read_req_d3[7:0], eth_read_req_d2};
end

// 把接收数据写入发送数据缓冲区
always @(posedge eth_read_clk or negedge sys_rst) begin 
	if(sys_rst == 1'b0) begin
		eth_read_len <= 11'd0;
		eth_read_offset <= 11'd0;
	end
	else if(eth_read_req_d1 && !eth_read_req_d2)begin
		eth_read_len <= eth_write_len;
		eth_read_offset <= eth_write_offset;
	end
end

// udp接收写请求cdc
cdc_pulse_toggle cdc_pulse_arpr(
	.src_clk(eth_write_clk),						 // arp接收时钟
	.src_rst(sys_rst),									 // arp接收复位
	.src_pulse(eth_write_req),					 // arp接收完成单脉冲
	.dst_clk(eth_read_clk),							 // arp发送时钟
	.dst_rst(sys_rst),									 // arp发送复位
	.dst_pulse(eth_read_req)						 // arp发送启动单脉冲
);

// udp发送读完成应答cdc
cdc_pulse_toggle cdc_pulse_arpt(
	.src_clk(eth_read_clk),						 	 // arp接收时钟
	.src_rst(sys_rst),									 // arp接收复位
	.src_pulse(eth_read_ack),					 	 // arp接收完成单脉冲
	.dst_clk(eth_write_clk),						 // arp发送时钟
	.dst_rst(sys_rst),									 // arp发送复位
	.dst_pulse(eth_write_ack)						 // arp发送启动单脉冲
);

	
// 以太网控制器
eth_controller eth_controller_inst(
	.eth_clk(sys_clk),					 		 		 // 以太网时钟 
	.eth_rst(sys_rst),						 			 // 以太网复位
	.eth_gtx_clk(pll_125mhz),			 	 		 // 以太网发送时钟gmii
	.eth_rxd_clk(pll_125mhz_90),		 		 // 以太网接收时钟
	.eth_rxd_data(eth_rxd_data),		 		 // 以太网接收数据
	.eth_rxd_valid(eth_rxd_valid),	 		 // 以太网接收有效
	.eth_txd_clk(eth_txd_clk),			 		 // 以太网发送时钟
	.eth_txd_data(eth_txd_data),		 		 // 以太网发送数据
	.eth_txd_en(eth_txd_en),			 	 		 // 以太网发送使能
	.eth_phy_rst(eth_phy_rst),			 		 // 以太网MAC复位
	.eth_phy_mdc(eth_phy_mdc),			 		 // 以太网MAC时钟
	.eth_phy_mdio(eth_phy_mdio),		 		 // 以太网MAC数据
	.eth_loc_port(SET_LOC_PORT),		 		 // 以太网本地端口
	.eth_loc_ip(SET_LOC_IP),  		 	 		 // 以太网本地IP
	.eth_loc_mac(SET_LOC_MAC),			 		 // 以太网本地MAC
	.eth_write_clk(eth_write_clk),		   // 以太网写bram时钟
	.eth_write_req(eth_write_req),		   // 以太网写bram请求
	.eth_write_ack(eth_write_ack),		   // 以太网写bram应答
	.eth_write_data(eth_write_data),	   // 以太网写bram数据
	.eth_write_cnt(eth_write_cnt),		   // 以太网写bram计数器
	.eth_write_offset(eth_write_offset), // 以太网写bram偏移双缓冲乒乓
	.eth_write_len(eth_write_len),		 	 // 以太网写bram数据大小
	.eth_write_addr(eth_write_addr),	 	 // 以太网写sdram地址用
	.eth_write_sel(),  	 								 // 以太网写sdram双缓冲乒乓同步
	.eth_write_en(eth_write_en),		 		 // 以太网写bram使能
	.eth_read_clk(eth_read_clk),		 		 // 以太网读时钟
	.eth_read_req(),		 	 					 		 // 以太网读请求(主动)
	.eth_read_start(eth_read_req_d3[8]),		 // 以太网读启动(被动)
	.eth_read_ack(eth_read_ack),     		 // 以太网读应答
	.eth_read_sel(),   	 								 // 以太网读sdram双缓冲乒乓同步
	.eth_read_data(eth_read_data),		 	 // 以太网读bram数据
	.eth_read_cnt(eth_read_cnt),     		 // 以太网读bram计数
	.eth_read_len(eth_read_len),		 		 // 以太网读bram长度
	.eth_read_offset(eth_read_offset),	 // 以太网读bram偏移双缓冲乒乓
	.eth_read_addr()		 								 // 以太网读sdram地址用
);

// 以太网接收数据缓存bram
alt_bram_8x2048 alt_bram_udpr(
	.data(eth_write_data),							 // udp接收写数据		
	.rdaddress(),												 // udp接收读地址
	.rdclock(),													 // udp接收读时钟
	.wraddress(eth_write_cnt),					 // udp接收写地址
	.wrclock(eth_write_clk),						 // udp接收写时钟
	.wren(eth_write_en),								 // udp接收写使能
	.q()																 // udp接收读数据	
);                                        
																				  
// 以太网发送数据缓存bram                    
alt_bram_8x2048 alt_bram_udpt(            
	.data(eth_write_data),							 // udp发送写数据		
	.rdaddress(eth_read_cnt),						 // udp发送读地址
	.rdclock(eth_read_clk),							 // udp发送读时钟
	.wraddress(eth_write_cnt),					 // udp发送写地址
	.wrclock(eth_write_clk),						 // udp发送写时钟
	.wren(eth_write_en),								 // udp发送写使能
	.q(eth_read_data)										 // udp发送读数据	
);

// 
endmodule
