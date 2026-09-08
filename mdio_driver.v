//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 mdio_driver
// 【功能描述】 mdio协议底层驱动
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.总线时钟频率1.25mhz
// 【说明】 3.写优先级高于读优先级
// 【说明】 4.读写数据采用线性序列机。
// 【说明】 5
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
module mdio_driver(
	input			 	 		  mdio_clk,					// 总线时钟 
	input      	 		  mdio_rst,					// 总线复位
	input      	 		  mdio_req_r,				// 总线请求读
	input             mdio_req_w,       // 总线请求写
	output reg [15:0] mdio_req_out,			// 总线请求输出
	input 		 [15:0] mdio_req_in,			// 总线请求输入 
	input 		 [ 4:0] mdio_req_addr,  	// 总线请求地址
	input 		 [ 4:0] mdio_phy_addr,    // 总线物理地址
	output reg 	 		  mdio_phy_clk,			// 总线物理时钟
	input      	 		  mdio_phy_in,			// 总线物理输入
	output reg        mdio_phy_out,			// 总线物理输出
	output reg        mdio_phy_oe,			// 总线物理使能
	output reg	 		  mdio_req_done			// 总线请求完成
);

// MDIO 状态机定义  
localparam STATE_IDLE  = 4'b0001;
localparam STATE_READ  = 4'b0010;
localparam STATE_WRITE = 4'b0100;
localparam STATE_END   = 4'b1000;

// MDIO总线参数
localparam SYSCLK_FREQ  = 50000000;  	// 系统时钟频率50mhz
localparam CFGMDC_FREQ  = 1250000;		// 总线时钟频率1.25mhz
localparam CFGMDC_CNT   = SYSCLK_FREQ / CFGMDC_FREQ - 1; 
localparam CFGMDC_PULSE = SYSCLK_FREQ / CFGMDC_FREQ / 2;

// MDIO总线参数
reg [ 3:0] mdio_state;
reg [ 3:0] mdio_state_next;
reg [ 5:0] mdio_clk_cnt;
reg [ 6:0] mdio_req_cnt;
reg        mdio_write_done;
reg        mdio_read_done;

// 状态寄存
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_state <= STATE_IDLE;
	else
		mdio_state <= mdio_state_next;
end

// 状态转移
always @(*) begin
	case(mdio_state)
		STATE_IDLE: begin
			if(mdio_req_w)
				mdio_state_next <= STATE_WRITE;
			else if(mdio_req_r)
				mdio_state_next <= STATE_READ;
			else
				mdio_state_next <= STATE_IDLE;
		end
		STATE_WRITE: begin
			if(mdio_write_done)
				mdio_state_next <= STATE_END;
			else
				mdio_state_next <= STATE_WRITE;
		end
		STATE_READ: begin
			if(mdio_read_done)	
				mdio_state_next <= STATE_END;
			else
				mdio_state_next <= STATE_READ;
		end
		STATE_END: begin
			mdio_state_next <= STATE_IDLE;
		end
		default: mdio_state_next <= STATE_IDLE;
	endcase
end

// 时钟分频计数器
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_clk_cnt <= 6'd0;
	else if(mdio_state== STATE_IDLE)
		mdio_clk_cnt <= 6'd0;
	else if(mdio_clk_cnt == CFGMDC_CNT)
		mdio_clk_cnt <= 6'd0;
	else
		mdio_clk_cnt <= mdio_clk_cnt + 1'b1;
end

// MDC输出时钟脉冲
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_phy_clk <= 1'b0;
	else if(mdio_state == STATE_IDLE)
		mdio_phy_clk <= 1'b0;
	else if(mdio_clk_cnt == 6'd0)
		mdio_phy_clk <= 1'b0;
	else if(mdio_clk_cnt == CFGMDC_PULSE)
		mdio_phy_clk <= 1'b1;
end

// MDIO输出使能
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_phy_oe <= 1'b0;
	else if(mdio_state == STATE_WRITE)
		mdio_phy_oe <= 1'b1;
	else if(mdio_req_cnt < 46)
		mdio_phy_oe <= 1'b1;
	else
		mdio_phy_oe <= 1'b0;
end

// MDIO写计数
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_req_cnt <= 7'd0;
	else if(mdio_state == STATE_IDLE)
		mdio_req_cnt <= 7'd0;
	else if(mdio_clk_cnt == CFGMDC_CNT)
		mdio_req_cnt <= mdio_req_cnt + 1'b1;
end

// MDIO写完成
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_write_done <= 1'b0;
	else if(mdio_state == STATE_WRITE && mdio_req_cnt == 7'd64)
		mdio_write_done <= 1'b1;
	else
		mdio_write_done <= 1'b0;
end

// MDIO数据输出
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_phy_out <= 1'b0;
	// 写REG寄存器
	else if(mdio_state == STATE_WRITE)begin
		case(mdio_req_cnt)
			7'd0:  mdio_phy_out <= 1'b1; //PRE
			7'd1:  mdio_phy_out <= 1'b1;
			7'd2:  mdio_phy_out <= 1'b1;
			7'd3:  mdio_phy_out <= 1'b1;
			7'd4:  mdio_phy_out <= 1'b1;
			7'd5:  mdio_phy_out <= 1'b1;
			7'd6:  mdio_phy_out <= 1'b1;
			7'd7:  mdio_phy_out <= 1'b1;
			7'd8:  mdio_phy_out <= 1'b1; 
			7'd9:  mdio_phy_out <= 1'b1;
			7'd10: mdio_phy_out <= 1'b1;
			7'd11: mdio_phy_out <= 1'b1;
			7'd12: mdio_phy_out <= 1'b1;
			7'd13: mdio_phy_out <= 1'b1;
			7'd14: mdio_phy_out <= 1'b1;
			7'd15: mdio_phy_out <= 1'b1;
			7'd16: mdio_phy_out <= 1'b1; 
			7'd17: mdio_phy_out <= 1'b1;
			7'd18: mdio_phy_out <= 1'b1;
			7'd19: mdio_phy_out <= 1'b1;
			7'd20: mdio_phy_out <= 1'b1;
			7'd21: mdio_phy_out <= 1'b1;
			7'd22: mdio_phy_out <= 1'b1;
			7'd23: mdio_phy_out <= 1'b1;
			7'd24: mdio_phy_out <= 1'b1; 
			7'd25: mdio_phy_out <= 1'b1;
			7'd26: mdio_phy_out <= 1'b1;
			7'd27: mdio_phy_out <= 1'b1;
			7'd28: mdio_phy_out <= 1'b1;
			7'd29: mdio_phy_out <= 1'b1;
			7'd30: mdio_phy_out <= 1'b1;
			7'd31: mdio_phy_out <= 1'b1;
			7'd32: mdio_phy_out <= 1'b0;//ST
			7'd33: mdio_phy_out <= 1'b1;
			7'd34: mdio_phy_out <= 1'b0;//OP
			7'd35: mdio_phy_out <= 1'b1;
			7'd36: mdio_phy_out <= mdio_phy_addr[4];//phy_addr
			7'd37: mdio_phy_out <= mdio_phy_addr[3];
			7'd38: mdio_phy_out <= mdio_phy_addr[2];
			7'd39: mdio_phy_out <= mdio_phy_addr[1];
			7'd40: mdio_phy_out <= mdio_phy_addr[0];
			7'd41: mdio_phy_out <= mdio_req_addr[4];//reg_addr
			7'd42: mdio_phy_out <= mdio_req_addr[3];
			7'd43: mdio_phy_out <= mdio_req_addr[2];
			7'd44: mdio_phy_out <= mdio_req_addr[1];
			7'd45: mdio_phy_out <= mdio_req_addr[0];
			7'd46: mdio_phy_out <= 1'b1;//TA
			7'd47: mdio_phy_out <= 1'b0;
			7'd48: mdio_phy_out <= mdio_req_in[15];//reg_data
			7'd49: mdio_phy_out <= mdio_req_in[14];
			7'd50: mdio_phy_out <= mdio_req_in[13];
			7'd51: mdio_phy_out <= mdio_req_in[12];
			7'd52: mdio_phy_out <= mdio_req_in[11];
			7'd53: mdio_phy_out <= mdio_req_in[10];
			7'd54: mdio_phy_out <= mdio_req_in[9];
			7'd55: mdio_phy_out <= mdio_req_in[8];
			7'd56: mdio_phy_out <= mdio_req_in[7];
			7'd57: mdio_phy_out <= mdio_req_in[6];
			7'd58: mdio_phy_out <= mdio_req_in[5];
			7'd59: mdio_phy_out <= mdio_req_in[4];
			7'd60: mdio_phy_out <= mdio_req_in[3];
			7'd61: mdio_phy_out <= mdio_req_in[2];
			7'd62: mdio_phy_out <= mdio_req_in[1];
			7'd63: mdio_phy_out <= mdio_req_in[0];
			7'd64: mdio_phy_out <= 1'b1;//IDLE
		endcase
	end
	// 读REG寄存器
	else if(mdio_state == STATE_READ) begin
		case(mdio_req_cnt)
			7'd0:  mdio_phy_out <= 1'b1; //PRE
			7'd1:  mdio_phy_out <= 1'b1;
			7'd2:  mdio_phy_out <= 1'b1;
			7'd3:  mdio_phy_out <= 1'b1;
			7'd4:  mdio_phy_out <= 1'b1;
			7'd5:  mdio_phy_out <= 1'b1;
			7'd6:  mdio_phy_out <= 1'b1;
			7'd7:  mdio_phy_out <= 1'b1;
			7'd8:  mdio_phy_out <= 1'b1; 
			7'd9:  mdio_phy_out <= 1'b1;
			7'd10: mdio_phy_out <= 1'b1;
			7'd11: mdio_phy_out <= 1'b1;
			7'd12: mdio_phy_out <= 1'b1;
			7'd13: mdio_phy_out <= 1'b1;
			7'd14: mdio_phy_out <= 1'b1;
			7'd15: mdio_phy_out <= 1'b1;
			7'd16: mdio_phy_out <= 1'b1; 
			7'd17: mdio_phy_out <= 1'b1;
			7'd18: mdio_phy_out <= 1'b1;
			7'd19: mdio_phy_out <= 1'b1;
			7'd20: mdio_phy_out <= 1'b1;
			7'd21: mdio_phy_out <= 1'b1;
			7'd22: mdio_phy_out <= 1'b1;
			7'd23: mdio_phy_out <= 1'b1;
			7'd24: mdio_phy_out <= 1'b1; 
			7'd25: mdio_phy_out <= 1'b1;
			7'd26: mdio_phy_out <= 1'b1;
			7'd27: mdio_phy_out <= 1'b1;
			7'd28: mdio_phy_out <= 1'b1;
			7'd29: mdio_phy_out <= 1'b1;
			7'd30: mdio_phy_out <= 1'b1;
			7'd31: mdio_phy_out <= 1'b1;
			7'd32: mdio_phy_out <= 1'b0;//ST
			7'd33: mdio_phy_out <= 1'b1;
			7'd34: mdio_phy_out <= 1'b1;//OP
			7'd35: mdio_phy_out <= 1'b0;
			7'd36: mdio_phy_out <= mdio_phy_addr[4];//phy_addr
			7'd37: mdio_phy_out <= mdio_phy_addr[3];
			7'd38: mdio_phy_out <= mdio_phy_addr[2];
			7'd39: mdio_phy_out <= mdio_phy_addr[1];
			7'd40: mdio_phy_out <= mdio_phy_addr[0];
			7'd41: mdio_phy_out <= mdio_req_addr[4];//reg_addr
			7'd42: mdio_phy_out <= mdio_req_addr[3];
			7'd43: mdio_phy_out <= mdio_req_addr[2];
			7'd44: mdio_phy_out <= mdio_req_addr[1];
			7'd45: mdio_phy_out <= mdio_req_addr[0];
			7'd46: mdio_phy_out <= 1'b0;//TA
			7'd47: mdio_phy_out <= 1'b0;
		endcase
	end
	else
		mdio_phy_out <= 1'b1;
end

// MDIO读完成
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_read_done <= 1'b0;
	else if(mdio_state == STATE_READ && mdio_req_cnt == 7'd64)
		mdio_read_done <= 1'b1;
	else
		mdio_read_done <= 1'b0;
end

// MDIO数据读取
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_req_out <= 16'd0;
	else if(mdio_state == STATE_READ && mdio_phy_clk == 0)begin
		case(mdio_req_cnt)
			7'd48: mdio_req_out[15] <= mdio_phy_in;//reg_data
			7'd49: mdio_req_out[14] <= mdio_phy_in;
			7'd50: mdio_req_out[13] <= mdio_phy_in;
			7'd51: mdio_req_out[12] <= mdio_phy_in;
			7'd52: mdio_req_out[11] <= mdio_phy_in;
			7'd53: mdio_req_out[10] <= mdio_phy_in;
			7'd54: mdio_req_out[9]  <= mdio_phy_in;
			7'd55: mdio_req_out[8]  <= mdio_phy_in;
			7'd56: mdio_req_out[7]  <= mdio_phy_in;
			7'd57: mdio_req_out[6]  <= mdio_phy_in;
			7'd58: mdio_req_out[5]  <= mdio_phy_in;
			7'd59: mdio_req_out[4]  <= mdio_phy_in;
			7'd60: mdio_req_out[3]  <= mdio_phy_in;
			7'd61: mdio_req_out[2]  <= mdio_phy_in;
			7'd62: mdio_req_out[1]  <= mdio_phy_in;
			7'd63: mdio_req_out[0]  <= mdio_phy_in;
		endcase
	end
end

// MDIO请求应答
always @(posedge mdio_clk or negedge mdio_rst) begin
	if(mdio_rst == 1'b0)
		mdio_req_done <= 1'b0;
	else if(mdio_state == STATE_END)
		mdio_req_done <= 1'b1;
	else
		mdio_req_done <= 1'b0;
end
	
endmodule
