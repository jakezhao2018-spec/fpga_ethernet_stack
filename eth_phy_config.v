//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_phy_config
// 【功能描述】 以太网PHY配置
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.复位等待时间必须严格按照手册要求，否则会导致复位失败。
// 【说明】 3.寄存器读写优先级，写优先级高，读优先级低。
// 【说明】 4.
// 【说明】 5.
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
module eth_phy_config(
	input			 cfg_clk,						// 配置时钟 
	input   	 cfg_rst,						// 配置复位
	output reg cfg_phy_rst,				// 配置MAC复位
	output  	 cfg_phy_mdc,				// 配置MAC时钟
	input   	 cfg_phy_mdio_in,		// 配置MAC数据
	output  	 cfg_phy_mdio_out,	// 配置MAC输出bit流
	output  	 cfg_phy_mdio_oe,   // 配置MAC输出使能
	output reg cfg_phy_done				// 配置MAC完成
);

// phy 状态机定义  
localparam STATE_INIT  = 5'b00001;
localparam STATE_IDLE  = 5'b00010;
localparam STATE_READ  = 5'b00100;
localparam STATE_WRITE = 5'b01000;
localparam STATE_END   = 5'b10000;
//
//localparam CFGPHY_REG1_DATA = 16'b0100_0001_0100_0000;  // MAC寄存器1参数回环
localparam CFGPHY_REG1_DATA = 16'b0001_0010_0000_0000;  // MAC寄存器1参数自动协商、重启自协商
localparam CFGPHY_REG1_ADDR = 5'd0;											// MAC地址
localparam CFGPHY_ADDR      = 5'd1;											// PHY地址
localparam CFGPHY_REG_CNT   = 4'd1;											// 寄存器个数
localparam CFGPHY_RST_MAX   = 23'd2000000;
localparam CFGPHY_RST_CNT   = 23'd500000;

// PHY配置参数
reg [ 4:0] cfg_state;
reg [ 4:0] cfg_state_next;
reg [ 2:0] cfg_reg_cnt;
reg [22:0] cfg_rst_cnt;
reg [15:0] cfg_reg_write;
reg [ 4:0] cfg_reg_addr;
reg [ 4:0] cfg_phy_addr;
reg [ 1:0] cfg_req_id;
reg        cfg_rst_done;
reg        cfg_read_done;
reg        cfg_write_done;
reg        cfg_write_req;
reg        cfg_read_req;
wire[15:0] cfg_reg_read;
wire       cfg_reg_done;
wire       cfg_reg_start;

// phy复位计数
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_rst_cnt <= 23'd0;
	else if(cfg_rst_cnt == CFGPHY_RST_MAX)
		cfg_rst_cnt <= 23'd0;
	else
		cfg_rst_cnt <= cfg_rst_cnt + 1'b1;
end

// phy复位信号输出
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_phy_rst <= 1'b0;
	else if(cfg_rst_cnt == CFGPHY_RST_CNT)
		cfg_phy_rst <= 1'b1;
end

// 初始化复位完成
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_rst_done <= 1'b0;
	else if(cfg_rst_cnt == CFGPHY_RST_MAX)
		cfg_rst_done <= cfg_phy_rst;
	else
		cfg_rst_done <= 1'b0;
end

// 状态寄存
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_state <= STATE_INIT;
	else
		cfg_state <= cfg_state_next;
end

// 状态转移
always @(*) begin
	case(cfg_state)
		STATE_INIT: begin
			if(cfg_rst_done)
				cfg_state_next <= STATE_IDLE;
			else
				cfg_state_next <= STATE_INIT;
		end
		STATE_IDLE: begin
			if(cfg_write_req)
				cfg_state_next <= STATE_WRITE;
			else if(cfg_read_req)
				cfg_state_next <= STATE_READ;
			else
				cfg_state_next <= STATE_IDLE;
		end
		STATE_WRITE: begin
			if(cfg_write_done)
				cfg_state_next <= STATE_END;
			else
				cfg_state_next <= STATE_WRITE;
		end
		STATE_READ: begin
			if(cfg_read_done)
				cfg_state_next <= STATE_END;
			else
				cfg_state_next <= STATE_READ;
		end
		STATE_END:
			cfg_state_next <= STATE_IDLE;
		default: 
			cfg_state_next <= STATE_INIT;
	endcase
end

// 配置phy地址
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_phy_addr <= 5'd0;
	else 
		cfg_phy_addr <= CFGPHY_ADDR;
end

// 写配置寄存器地址
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_reg_addr <= 5'd0;
	else if(cfg_state == STATE_WRITE)
		cfg_reg_addr <= 5'd0; // reg1
	else if(cfg_state == STATE_READ)
		cfg_reg_addr <= 5'd17;
end

// 写寄存器计数
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_reg_cnt <= 3'd0;
	else if(cfg_state == STATE_END) begin
		if(cfg_reg_cnt < CFGPHY_REG_CNT)
			cfg_reg_cnt <= cfg_reg_cnt + 1'b1;
	end
end

// 写配置寄存器
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_reg_write <= 16'd0;
	else begin
		case(cfg_reg_cnt)
			0: cfg_reg_write <= CFGPHY_REG1_DATA;
			default: cfg_reg_write <= CFGPHY_REG1_DATA;
		endcase
	end
end

// 写请求
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_write_req <= 1'b1;
	else if(cfg_state == STATE_WRITE)
		cfg_write_req <= 1'b0;
end

// 读请求
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_read_req <= 1'b1;
	else if(cfg_state == STATE_READ)
		cfg_read_req <= 1'b0;
end

// 请求id
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_req_id <= 2'b0;
	else if(cfg_state != STATE_INIT)
		cfg_req_id <= {cfg_write_req, cfg_read_req};
	else
		cfg_req_id <= 2'b0;
end

// 写请求完成
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_write_done <= 1'b0;
	else if(cfg_state == STATE_WRITE)
		cfg_write_done <= cfg_reg_done;
	else
		cfg_write_done <= 1'b0;
end

// 读请求完成
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_read_done <= 1'b0;
	else if(cfg_state == STATE_READ)
		cfg_read_done <= cfg_reg_done;
	else
		cfg_read_done <= 1'b0;
end

// 配置phy完成 后续可以增加网络状态实时检测
always @(posedge cfg_clk or negedge cfg_rst) begin
	if(cfg_rst == 1'b0)
		cfg_phy_done <= 1'b0;
	else if(cfg_state == STATE_READ && cfg_read_done)
		cfg_phy_done <= 1'b1;
end

// mdio总线驱动
mdio_driver mdio_driver_inst(
	.mdio_clk(cfg_clk),								// 总线时钟 
	.mdio_rst(cfg_rst),								// 总线复位
	.mdio_req_r(cfg_req_id[0]),				// 总线请求读
	.mdio_req_w(cfg_req_id[1]),     	// 总线请求写
	.mdio_req_out(cfg_reg_read),			// 总线寄存器输出数据
	.mdio_req_in(cfg_reg_write),			// 总线寄存器输入数据 
	.mdio_req_addr(cfg_reg_addr), 		// 总线寄存器地址
	.mdio_phy_addr(cfg_phy_addr),   	// 总线物理地址
	.mdio_phy_clk(cfg_phy_mdc),				// 总线物理时钟
	.mdio_phy_in(cfg_phy_mdio_in),		// 总线物理输入
	.mdio_phy_out(cfg_phy_mdio_out),	// 总线物理输出
	.mdio_phy_oe(cfg_phy_mdio_oe),		// 总线物理使能
	.mdio_req_done(cfg_reg_done)			// 总线请求完成
);

endmodule
