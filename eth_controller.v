//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_controller
// 【功能描述】 以太网协议栈主控制器（高并发写入BRAM + 发送优先级仲裁）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.以太网帧接收高并发写入arp、icmp、udp bram缓存
// 【说明】 3.arp、icmp、udp收到帧接收完成信号开始读取bram缓存
// 【说明】 4.发送仲裁优先级arp、icmp、udp
// 【说明】 5.udp发送是从模式，必须等待接收解析到主机ip、mac、port才能发送
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
module eth_controller(
	input				 eth_clk,					 // 以太网时钟 
	input        eth_rst,					 // 以太网复位
	input        eth_gtx_clk,			 // 以太网发送时钟gmii
	input        eth_rxd_clk,			 // 以太网接收时钟
	input  [3:0] eth_rxd_data,		 // 以太网接收数据
	input   		 eth_rxd_valid,		 // 以太网接收有效
	output 			 eth_txd_clk,			 // 以太网发送时钟
	output [3:0] eth_txd_data,		 // 以太网发送数据
	output			 eth_txd_en,			 // 以太网发送使能
	output			 eth_phy_rst,			 // 以太网MAC复位
	output       eth_phy_mdc,			 // 以太网MAC时钟
	inout        eth_phy_mdio,		 // 以太网MAC数据
	input  [15:0]eth_loc_port,		 // 以太网本地端口
	input  [31:0]eth_loc_ip,  		 // 以太网本地IP
	input  [47:0]eth_loc_mac,			 // 以太网本地MAC
	output       eth_write_clk,		 // 以太网写bram时钟
	output       eth_write_req,		 // 以太网写bram请求
	input        eth_write_ack,		 // 以太网写bram应答
	output [ 7:0]eth_write_data,	 // 以太网写bram数据
	output [ 8:0]eth_write_cnt,		 // 以太网写bram计数器
	output [ 8:0]eth_write_offset, // 以太网写bram偏移双缓冲乒乓
	output [ 8:0]eth_write_len,		 // 以太网写bram数据大小
	output [23:0]eth_write_addr,	 // 以太网写sdram地址用
	output       eth_write_sel,  	 // 以太网写sdram双缓冲乒乓同步
	output       eth_write_en,		 // 以太网写bram使能
	output       eth_read_clk,		 // 以太网读时钟
	output       eth_read_req,		 // 以太网读请求(主动)
	input        eth_read_start,   // 以太网读启动(被动)
	output       eth_read_ack,     // 以太网读应答
	input        eth_read_sel,   	 // 以太网读sdram双缓冲乒乓同步
	input  [ 7:0]eth_read_data,		 // 以太网读bram数据
	output [10:0]eth_read_cnt,     // 以太网读bram计数
	input  [10:0]eth_read_len,		 // 以太网读bram长度
	input  [10:0]eth_read_offset,	 // 以太网读bram偏移双缓冲乒乓
	output [23:0]eth_read_addr		 // 以太网读sdram地址用
);

// ETH 状态机定义  
localparam STATE_IDLE = 5'b00001;
localparam STATE_INIT = 5'b00010;
localparam STATE_ARP  = 5'b00100;
localparam STATE_ICMP = 5'b01000;
localparam STATE_UDP  = 5'b10000;

// SDRAM主控制器参数
reg [ 4:0]  eth_state;
reg [ 4:0]  eth_state_next;
reg         eth_task_req_arp;
reg         eth_task_req_icmp;
reg         eth_task_req_udp;
reg         eth_task_busy;

// 以太网帧接收写入bram缓存
wire [ 7:0] eth_frame_write_data;
wire [10:0] eth_frame_write_addr;
wire [10:0] eth_frame_write_len;
wire        eth_frame_write_en;
wire        eth_frame_rx_done;

// 以太网帧发送读bram缓存
reg  [ 7:0] eth_frame_read_data;
reg  [10:0] eth_frame_read_len;
wire [10:0] eth_frame_read_addr;
reg         eth_frame_tx_start;
wire        eth_frame_tx_done;

// arp接收读frame_rx->bram
wire [ 7:0] arpr_read_data;
wire [ 5:0] arpr_read_addr;
wire [ 5:0] arpr_read_len;
wire        arpr_read_done;
wire [31:0] arpr_dst_ip;
wire [47:0] arpr_dst_mac;

// arp发送封包写bram->frame_tx
wire        arpt_write_start;
wire [ 7:0] arpt_write_data;
wire [ 5:0] arpt_write_addr;
wire [ 5:0] arpt_write_len;
wire        arpt_write_en;
wire        arpt_write_done;

// 以太网帧发送arp数据包读bram
reg  [ 5:0] arpt_frame_read_addr;     
wire [ 7:0] arpt_frame_read_data;

// icmp接收读frame_rx->bram
wire [ 7:0] icmpr_read_data;
wire [ 7:0] icmpr_read_addr;
wire [ 7:0] icmpr_read_len;
wire        icmpr_read_done;
wire [31:0] icmpr_dst_ip;
wire [47:0] icmpr_dst_mac;

// icmp接收返回参数写入bram
wire [ 7:0] icmpr_write_data;
wire [ 7:0] icmpr_write_addr;
wire [ 7:0] icmpr_write_len;
wire        icmpr_write_en;

// icmp发送读icmp返回参数
wire [ 7:0] icmpt_read_data;
wire [ 7:0] icmpt_read_addr;
wire [ 7:0] icmpt_read_len;
wire        icmpt_read_done;

// icmp发送封包写bram->frame_tx
wire        icmpt_write_start;
wire [ 7:0] icmpt_write_data;
wire [ 7:0] icmpt_write_addr;
wire [ 7:0] icmpt_write_len;
wire        icmpt_write_done;
wire        icmpt_write_en;

// 以太网帧发送icmp数据包读bram
reg  [ 7:0] icmpt_frame_read_addr;
wire [ 7:0] icmpt_frame_read_data;

// udp接收读取bram解包
wire [ 7:0] udpr_read_data;
wire [10:0] udpr_read_addr;
wire [10:0] udpr_read_len;
wire        udpr_read_done;
wire [31:0] udpr_dst_ip;
wire [47:0] udpr_dst_mac;
wire [15:0] udpr_dst_port;
wire        udpr_send_ready;

// udp发送封包写bram
reg         udpt_write_start;
wire [ 7:0] udpt_write_data;
wire [10:0] udpt_write_addr;
wire [10:0] udpt_write_len;
wire        udpt_write_done;
wire        udpt_write_en;

// 以太网帧发送udp数据包读bram
reg  [10:0] udps_read_addr;
wire [ 7:0] udps_read_data;

// 以太网接收单沿8bit
wire [ 7:0] eth_gmii_rxd_data;
wire        eth_gmii_rxd_valid;
wire        eth_gmii_rxd_clk;

// 以太网发送单沿8bit
wire [ 7:0] eth_gmii_txd_data;
wire        eth_gmii_txd_en;
wire        eth_gmii_txd_clk;  

// 以太网配置
wire       eth_phy_done;
reg        eth_phy_done_d1;
reg        eth_phy_done_d2;      

// 三态门
wire       eth_phy_mdio_in;
wire       eth_phy_mdio_out;
wire       eth_phy_mdio_oe;
	
//三态门控制
assign eth_phy_mdio    = eth_phy_mdio_oe ? eth_phy_mdio_out : 1'bz;
assign eth_phy_mdio_in = eth_phy_mdio;

// 一段式寄存器同步
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_state <= STATE_INIT;
	else
		eth_state <= eth_state_next;
end

// 二段式状态跳转
always @(*) begin
	case(eth_state)
		STATE_INIT: begin
			if(eth_phy_done_d2)
				eth_state_next = STATE_IDLE;
			else
				eth_state_next = STATE_INIT;
		end
		STATE_IDLE: begin
			if(eth_task_req_arp)
				eth_state_next = STATE_ARP;
			else if(eth_task_req_icmp)
				eth_state_next = STATE_ICMP;
			else if(eth_task_req_udp)
				eth_state_next = STATE_UDP;	
			else
				eth_state_next = STATE_IDLE;	
		end
		STATE_ARP: begin
			if(eth_frame_tx_done)
				eth_state_next = STATE_IDLE;
			else 
				eth_state_next = STATE_ARP;
		end
		STATE_ICMP: begin
			if(eth_frame_tx_done)
				eth_state_next = STATE_IDLE;
			else 
				eth_state_next = STATE_ICMP;
		end
		STATE_UDP: begin
			if(eth_frame_tx_done)
				eth_state_next = STATE_IDLE;
			else 
				eth_state_next = STATE_UDP;
		end
		default: 
			eth_state_next = STATE_INIT;
	endcase
end

// 以太网初始化完成打拍
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0) begin
		eth_phy_done_d1 <= 1'b0;
		eth_phy_done_d2 <= 1'b0;
	end
	else begin
		eth_phy_done_d1 <= eth_phy_done;
		eth_phy_done_d2 <= eth_phy_done_d1;
	end
end

// 以太网帧发送启动
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_frame_tx_start <= 1'b0;
	else if(eth_state != STATE_IDLE)
		eth_frame_tx_start <= eth_task_busy ? 1'b0 : 1'b1;
	else 
		eth_frame_tx_start <= 1'b0;
end

// 以太网发送任务忙
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_task_busy <= 1'b0;
	else if(eth_state == STATE_IDLE)
		eth_task_busy <= 1'b0;
	else
		eth_task_busy <= 1'b1;
end

// 以太网任务arp发送请求锁定，任务执行后释放
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_task_req_arp <= 1'b0;
	else if(arpt_write_done)
		eth_task_req_arp <= 1'b1;
	else if(eth_state == STATE_ARP)
		eth_task_req_arp <= 1'b0;
end

// 以太网任务icmp发送请求锁定，任务执行后释放
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_task_req_icmp <= 1'b0;
	else if(icmpt_write_done)
		eth_task_req_icmp <= 1'b1;
	else if(eth_state == STATE_ICMP)
		eth_task_req_icmp <= 1'b0;
end

// 以太网任务udp送请求锁定，任务执行后释放
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_task_req_udp <= 1'b0;
	else if(udpt_write_done)
		eth_task_req_udp <= 1'b1;
	else if(eth_state == STATE_UDP)
		eth_task_req_udp <= 1'b0;
end

// 以太网帧发送读数据源通道切换
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_frame_read_data <= 8'd0;
	else begin
		case(eth_state)
			STATE_ARP:  eth_frame_read_data <= arpt_frame_read_data;
			STATE_ICMP: eth_frame_read_data <= icmpt_frame_read_data;
			STATE_UDP:  eth_frame_read_data <= udps_read_data;
			default:    eth_frame_read_data <= eth_frame_read_data;
		endcase
	end
end

// 以太网帧发送读数据长度源通道切换
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		eth_frame_read_len <= 11'd0;
	else begin
		case(eth_state)
			STATE_ARP:  eth_frame_read_len <= {5'b0, arpt_write_len};
			STATE_ICMP: eth_frame_read_len <= {3'b0, icmpt_write_len};
			STATE_UDP:  eth_frame_read_len <= udpt_write_len;
			default:    eth_frame_read_len <= eth_frame_read_len;
		endcase
	end
end

// 以太网帧发送读arp地址
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0) 
		arpt_frame_read_addr <= 6'd0;
	else if(eth_state == STATE_ARP)
		arpt_frame_read_addr <= eth_frame_read_addr[5:0];
end

// 以太网帧发送读icmp地址
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0) 
		icmpt_frame_read_addr <= 8'd0;
	else if(eth_state == STATE_ICMP)
		icmpt_frame_read_addr <= eth_frame_read_addr[7:0];
end

// 以太网帧发送读udp地址
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0) 
		udps_read_addr <= 11'd0;
	else if(eth_state == STATE_UDP)
		udps_read_addr <= eth_frame_read_addr;
end

// arp接收完成->arp发送启动
cdc_pulse_toggle cdc_pulse_arpr(
	.src_clk(eth_gmii_rxd_clk),								// arp接收时钟
	.src_rst(eth_rst),												// arp接收复位
	.src_pulse(arpr_read_done),								// arp接收完成单脉冲
	.dst_clk(eth_gtx_clk),										// arp发送时钟
	.dst_rst(eth_rst),												// arp发送复位
	.dst_pulse(arpt_write_start)							// arp发送启动单脉冲
);

// icmp接收完成->icmp发送启动
cdc_pulse_toggle cdc_pulse_icmpr(
	.src_clk(eth_gmii_rxd_clk),								// icmp接收时钟
	.src_rst(eth_rst),												// icmp接收复位
	.src_pulse(icmpr_read_done),							// icmp接收完成单脉冲
	.dst_clk(eth_gtx_clk),										// icmp发送时钟
	.dst_rst(eth_rst),												// icmp发送复位
	.dst_pulse(icmpt_write_start)							// icmp发送启动单脉冲
);

// 以太网MAC配置
eth_phy_config eth_phy_config_inst(
	.cfg_clk(eth_clk),												// 配置时钟 
	.cfg_rst(eth_rst),												// 配置复位
	.cfg_phy_rst(eth_phy_rst),								// 配置MAC复位
	.cfg_phy_mdc(eth_phy_mdc),								// 配置MAC时钟
	.cfg_phy_mdio_in(eth_phy_mdio_in),				// 配置MAC数据
	.cfg_phy_mdio_out(eth_phy_mdio_out),			// 配置MAC输出bit流
	.cfg_phy_mdio_oe(eth_phy_mdio_oe),   			// 配置MAC输出使能
	.cfg_phy_done(eth_phy_done)								// 配置MAC完成
);

/**
 *@功能:以太网接收
 */ 
// 以太网接收双沿转换单沿
ethr_rgmii_to_gmii ethr_rgmii_to_gmii_inst(
	.ethr_rgmii_clk(eth_rxd_clk),							// 以太网rgmii时钟
	.ethr_rgmii_data(eth_rxd_data),						// 以太网rgmii数据
	.ethr_rgmii_valid(eth_rxd_valid),					// 以太网rgmii有效
	.ethr_gmii_clk(eth_gmii_rxd_clk),					// 以太网gmii 时钟
	.ethr_gmii_data(eth_gmii_rxd_data),				// 以太网gmii 数据
	.ethr_gmii_valid(eth_gmii_rxd_valid)	  	// 以太网gmii 有效
);

// 以太网接收帧并行分发
eth_frame_rxd eth_frame_rxd_inst(
	.ethr_clk(eth_gmii_rxd_clk),							// 以太网时钟 
	.ethr_rst(eth_rst),												// 以太网复位
	.ethr_rxd_data(eth_gmii_rxd_data),				// 以太网接收数据
	.ethr_rxd_valid(eth_gmii_rxd_valid),			// 以太网接收有效
	.ethr_write_data(eth_frame_write_data), 	// 以太网帧写数据
	.ethr_write_len(eth_frame_write_len),			// 以太网帧写长度
	.ethr_write_addr(eth_frame_write_addr),		// 以太网帧写地址
	.ethr_write_en(eth_frame_write_en),				// 以太网帧写使能
	.ethr_write_done(eth_frame_rx_done)				// 以太网帧写完成
);

// 接收完整arp帧写入bram
alt_bram_8x64 alt_bram_arpr(
	.data(eth_frame_write_data),							// 以太网帧接收写数据
	.rdaddress(arpr_read_addr),								// 以太网帧arp读地址
	.rdclock(eth_gmii_rxd_clk),								// 以太网帧接收读时钟
	.wraddress(eth_frame_write_addr[5:0]),		// 以太网帧接收写地址
	.wrclock(eth_gmii_rxd_clk),								// 以太网帧接收写时钟
	.wren(eth_frame_write_en),								// 以太网帧接收写使能
	.q(arpr_read_data)												// 以太网帧arp读数据
);

// 接收完整icmp帧写入bram
alt_bram_8x256 alt_bram_icmpr(
	.data(eth_frame_write_data),							// 以太网帧接收写数据			
	.rdaddress(icmpr_read_addr),							// 以太网帧icmp读地址				
	.rdclock(eth_gmii_rxd_clk),								// 以太网帧接收读时钟			
	.wraddress(eth_frame_write_addr[7:0]),		// 以太网帧接收写地址		
	.wrclock(eth_gmii_rxd_clk),								// 以太网帧接收写时钟			
	.wren(eth_frame_write_en),								// 以太网帧接收写使能			
	.q(icmpr_read_data)												// 以太网帧icmp读数据					
);

// 接收icmp帧返回参数写入bram
alt_bram_8x256 alt_bram_icmpw(
	.data(icmpr_write_data),									// 以太网帧接收写数据			
	.rdaddress(icmpt_read_addr),							// 以太网帧icmp读地址				
	.rdclock(eth_gtx_clk),										// 以太网帧发送读时钟			
	.wraddress(icmpr_write_addr),							// 以太网帧接收写地址		
	.wrclock(eth_gmii_rxd_clk),								// 以太网帧接收写时钟			
	.wren(icmpr_write_en),										// 以太网帧接收写使能			
	.q(icmpt_read_data)												// 以太网帧icmp读数据					
);

// 接收完整udp帧写入bram
alt_bram_8x2048 alt_bram_udpr(
	.data(eth_frame_write_data),							// 以太网帧接收写数据			
	.rdaddress(udpr_read_addr),								// 以太网帧udp读地址				
	.rdclock(eth_gmii_rxd_clk),								// 以太网帧接收读时钟			
	.wraddress(eth_frame_write_addr),					// 以太网帧接收写地址		
	.wrclock(eth_gmii_rxd_clk),								// 以太网帧接收写时钟			
	.wren(eth_frame_write_en),								// 以太网帧接收写使能			
	.q(udpr_read_data)												// 以太网帧udp读数据					
);

// arp接收读取bram
arp_receive arp_receive_inst(
	.arpr_clk(eth_gmii_rxd_clk),							// arp时钟 
	.arpr_rst(eth_rst),												// arp复位
	.arpr_loc_ip(eth_loc_ip),									// arp本地IP
	.arpr_loc_mac(eth_loc_mac),								// arp本地mac
	.arpr_dst_ip(arpr_dst_ip),								// arp目标IP
	.arpr_dst_mac(arpr_dst_mac),							// arp目标mac
	.arpr_read_start(eth_frame_rx_done),			// arp读启动
	.arpr_read_data(arpr_read_data),					// arp读数据
	.arpr_read_len(eth_frame_write_len[5:0]),	// arp读长度
	.arpr_read_addr(arpr_read_addr),					// arp读计数
	.arpr_read_done(arpr_read_done)						// arp读完成
);

// icmp接收读取bram
icmp_receive icmp_receive_isnt(
	.icmpr_clk(eth_gmii_rxd_clk),							// icmpr时钟 
	.icmpr_rst(eth_rst),											// icmpr复位
	.icmpr_loc_ip(eth_loc_ip),								// icmpr本地IP
	.icmpr_loc_mac(eth_loc_mac),							// icmpr本地mac
	.icmpr_dst_ip(icmpr_dst_ip),							// icmpr目标IP
	.icmpr_dst_mac(icmpr_dst_mac),						// icmpr目标mac
	.icmpr_read_start(eth_frame_rx_done),			// icmpr读启动
	.icmpr_read_data(icmpr_read_data),				// icmpr读数据
	.icmpr_read_len(eth_frame_write_len[7:0]),// icmpr读长度
	.icmpr_read_addr(icmpr_read_addr),				// icmpr读计数
	.icmpr_write_data(icmpr_write_data),			// icmpr写数据
	.icmpr_write_len(icmpr_write_len),	  		// icmpr写长度
	.icmpr_write_addr(icmpr_write_addr),			// icmpr写地址
	.icmpr_write_en(icmpr_write_en),					// icmpr写使能
	.icmpr_read_done(icmpr_read_done)					// icmpr读完成
);

// udp接收读取bram
udp_receive udp_receive_inst(
	.udpr_clk(eth_gmii_rxd_clk),							// udpr时钟 
	.udpr_rst(eth_rst),												// udpr复位
	.udpr_loc_port(eth_loc_port),							// udpr本地端口
	.udpr_loc_ip(eth_loc_ip),									// udpr本地IP
	.udpr_loc_mac(eth_loc_mac),								// udpr本地mac
	.udpr_dst_ip(udpr_dst_ip),								// udpr源IP
	.udpr_dst_mac(udpr_dst_mac),							// udpr源mac
	.udpr_dst_port(udpr_dst_port),						// udpr源端口
	.udpr_read_start(eth_frame_rx_done),			// udpr读启动
	.udpr_read_data(udpr_read_data),					// udpr读数据
	.udpr_read_len(eth_frame_write_len),			// udpr读长度
	.udpr_read_addr(udpr_read_addr),					// udpr读计数
	.udpr_write_clk(eth_write_clk),     			// udpr写时钟
	.udpr_write_req(eth_write_req),						// udpr写请求
	.udpr_write_ack(eth_write_ack),						// udpr写响应
	.udpr_write_data(eth_write_data),	  			// udpr写数据
	.udpr_write_len(eth_write_len),	  				// udpr写长度
	.udpr_write_cnt(eth_write_cnt),	  				// udpr写计数
	.udpr_write_offset(eth_write_offset),			// udpr写偏移
	.udpr_write_addr(eth_write_addr),					// udpr写地址sdram
	.udpr_write_en(eth_write_en),							// udpr写使能
	.udpr_write_sel(eth_write_sel),						// udpr写缓冲区乒乓
	.udpr_send_ready(udpr_send_ready),				// udpr发送就绪
	.udpr_read_done(udpr_read_done)						// udpr读完成
);

/**
 *@功能:以太网发送
 */ 
// 以太网发送单沿转换双沿
etht_gmii_to_rgmii etht_gmii_to_rgmii_inst(
	.etht_gmii_clk(eth_gtx_clk),							// 以太网发送gmii时钟
	.etht_gmii_data(eth_gmii_txd_data),				// 以太网发送gmii数据
	.etht_gmii_en(eth_gmii_txd_en),			  		// 以太网发送gmii使能
	.etht_rgmii_clk(eth_txd_clk),		  				// 以太网发送rgmii 时钟
	.etht_rgmii_data(eth_txd_data),						// 以太网发送rgmii 数据
	.etht_rgmii_en(eth_txd_en)		 	  				// 以太网发送rgmii 使能
);

// 以太网帧发送读BRAM
eth_frame_txd eth_frame_txd_inst(
	.etht_clk(eth_gtx_clk),										// eth时钟 
	.etht_rst(eth_rst),												// eth复位
	.etht_txd_data(eth_gmii_txd_data),				// eth发送数据
	.etht_txd_en(eth_gmii_txd_en),						// eth发送使能
	.etht_txd_done(eth_frame_tx_done),				// eth发送完成
	.etht_read_start(eth_frame_tx_start),			// eth发送启动
	.etht_read_data(eth_frame_read_data),			// eth读数据
	.etht_read_len(eth_frame_read_len),				// eth读长度
	.etht_read_addr(eth_frame_read_addr)			// eth读地址
);

// arp发送数据封包写入RRAM
arp_send arp_send_inst(
	.arpt_clk(eth_gtx_clk),					 					// arp时钟 
	.arpt_rst(eth_rst),					 	  					// arp复位
	.arpt_loc_ip(eth_loc_ip),			  					// arp本地IP
	.arpt_loc_mac(eth_loc_mac),			 					// arp本地mac
	.arpt_dst_ip(arpr_dst_ip),			 	 				// arp目标IP
	.arpt_dst_mac(arpr_dst_mac),			 				// arp目标mac
	.arpt_write_start(arpt_write_start),			// arp写启动
	.arpt_write_data(arpt_write_data),	 			// arp写数据
	.arpt_write_len(arpt_write_len),		 			// arp写长度
	.arpt_write_addr(arpt_write_addr),	 			// arp写请求
	.arpt_write_en(arpt_write_en),		 				// arp写使能
	.arpt_write_done(arpt_write_done) 	 			// arp写完成
);

// icmp发送数据封包写入BRAM
icmp_send icmp_send_inst(
	.icmpt_clk(eth_gtx_clk),					 				// icmp时钟 
	.icmpt_rst(eth_rst),					 						// icmp复位
	.icmpt_loc_ip(eth_loc_ip),				 				// icmp本地IP
	.icmpt_loc_mac(eth_loc_mac),			 				// icmp本地mac
	.icmpt_dst_ip(icmpr_dst_ip),			 	 			// icmp目标IP
	.icmpt_dst_mac(icmpr_dst_mac),			 			// icmp目标mac
	.icmpt_read_data(icmpt_read_data),	 			// icmp读数据
	.icmpt_read_len(icmpr_write_len),		 			// icmp读长度
	.icmpt_read_addr(icmpt_read_addr),	 			// icmp读地址
	.icmpt_write_start(icmpt_write_start),		// icmp写启动
	.icmpt_write_data(icmpt_write_data),			// icmp写数据
	.icmpt_write_len(icmpt_write_len),				// icmp写长度
	.icmpt_write_addr(icmpt_write_addr),			// icmp写地址
	.icmpt_write_en(icmpt_write_en),		 			// icmp写使能
	.icmpt_write_done(icmpt_write_done)	 			// icmp写完成
);

// udp发送数据封包写入BRAM
udp_send udp_send_inst(
	.udpt_clk(eth_gtx_clk),					 					// udp时钟 
	.udpt_rst(eth_rst),					 							// udp复位
	.udpt_loc_port(eth_loc_port),		 					// udp本地端口
	.udpt_loc_ip(eth_loc_ip),			 						// udp本地IP
	.udpt_loc_mac(eth_loc_mac),			 					// udp本地mac
	.udpt_dst_ip(udpr_dst_ip),			 	 				// udp源IP
	.udpt_dst_mac(udpr_dst_mac),			 				// udp源mac
	.udpt_dst_port(udpr_dst_port),		 				// udp源端口
	.udpt_read_data(eth_read_data),	 	  			// udp读数据
	.udpt_read_len(eth_read_len),		 					// udp读长度
	.udpt_read_offset(eth_read_offset),   		// udp读偏移双缓冲乒乓
	.udpt_read_cnt(eth_read_cnt),	 	 					// udp读计数
	.udpt_read_addr(eth_read_addr),	 	 				// udp读地址
	.udpt_read_clk(eth_read_clk),							// udp读时钟
	.udpt_write_start(udpt_write_start),			// udp写启动
	.udpt_write_data(udpt_write_data),	  		// udp写数据
	.udpt_write_len(udpt_write_len),		  		// udp写长度
	.udpt_write_addr(udpt_write_addr),	  		// udp写地址
	.udpt_write_en(udpt_write_en),		 				// udp写使能
	.udpt_write_done(udpt_write_done) 	  		// udp写完成
);

// 发送完整arp帧写入bram缓存
alt_bram_8x64 alt_bram_arpt(
	.data(arpt_write_data),										// arp发送写数据		
	.rdaddress(arpt_frame_read_addr),					// arp发送帧读地址  
	.rdclock(eth_gtx_clk),										// arp发送帧读时钟
	.wraddress(arpt_write_addr),							// arp发送写地址
	.wrclock(eth_gtx_clk),										// arp发送写时钟
	.wren(arpt_write_en),											// arp发送写使能
	.q(arpt_frame_read_data)									// arp发送帧读数据
);

// 发送完整icmp帧写入bram缓存
alt_bram_8x256 alt_bram_icmpt(
	.data(icmpt_write_data),                  // icmp发送写数据		
	.rdaddress(icmpt_frame_read_addr),        // icmp发送帧读地址    
	.rdclock(eth_gtx_clk),                    // icmp发送帧读时钟   
	.wraddress(icmpt_write_addr),             // icmp发送写地址       
	.wrclock(eth_gtx_clk),                    // icmp发送写时钟      
	.wren(icmpt_write_en),                    // icmp发送写使能      
	.q(icmpt_frame_read_data)                 // icmp发送帧读数据           
);

// 发送完整udp帧写入bram缓存
alt_bram_8x2048 alt_bram_udpt(
	.data(udpt_write_data),										// udp发送写数据		
	.rdaddress(udps_read_addr),								// udp发送帧读地址  
	.rdclock(eth_gtx_clk),										// udp发送帧读时钟
	.wraddress(udpt_write_addr),							// udp发送写地址
	.wrclock(eth_gtx_clk),										// udp发送写时钟
	.wren(udpt_write_en),											// udp发送写使能
	.q(udps_read_data)												// udp发送帧读数据	
);

// udp发送请求必须等待发送就绪
assign eth_read_ack = udpt_write_done;
always @(posedge eth_gtx_clk or negedge eth_rst) begin
	if(eth_rst == 1'b0)
		udpt_write_start <= 1'b0;
	else if(udpr_send_ready)
		udpt_write_start <= eth_read_start;
	else
		udpt_write_start <= eth_read_req;
end
endmodule
