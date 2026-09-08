//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_check_sum
// 【功能描述】 以太网协议栈校验累加和
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.把8bit数据拼接成16bit
// 【说明】 2.
// 【说明】 3.
// 【说明】 4.
// 【说明】 5
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
module eth_check_sum( 
	input				 			check_clk,				// sum校验时钟 
	input        			check_rst,				// sum校验复位
	input      [ 7:0] check_sum_in,		 	// sum校验输入
	input         		check_sum_en,		 	// sum校验使能
	input         		check_sum_val,    // sum校验初始化 
	output reg [15:0] check_sum_out     // sum校验输出
);

//
reg 			 check_flip;
reg [31:0] check_data;
reg [15:0] check_buf;
reg [16:0] check_stage;

// 校验信号翻转一圈单转双
always @(posedge check_clk or negedge check_rst) begin
	if(check_rst == 1'b0)
		check_flip <= 1'b0;
	else if(check_sum_val)
		check_flip <= 1'b0;
	else if(check_sum_en)
		check_flip <= !check_flip;
end
 
// 校验输入单子字节拼双字节
always @(posedge check_clk or negedge check_rst) begin
	if(check_rst == 1'b0)
		check_buf <= 16'd0;
	else if(check_sum_val)
		check_buf <= 16'd0;
	else if(check_sum_en)
		check_buf <= {check_buf[7:0], check_sum_in};
	else
		check_buf <= {check_buf[7:0], 8'd0};
end

// 同步计算双字节累计加和
always @(posedge check_clk or negedge check_rst) begin
	if(check_rst == 1'b0)
		check_data <= 32'd0;
	else if(check_sum_val)
		check_data <= 32'd0;
	else if(check_sum_en && check_flip == 1'b0)
		check_data <= check_data + {16'd0, check_buf};
end

// 插入一级流水线寄存器
always @(posedge check_clk or negedge check_rst) begin
	if(check_rst == 1'b0)
		check_stage <= 17'd0;
	else if(check_sum_val)
		check_stage <= 17'd0;
	else
		check_stage <= check_data[15:0] + check_data[31:16];
end

// 校验寄存器打拍输出，切断组合逻辑路径
always @(posedge check_clk or negedge check_rst) begin
	if(check_rst == 1'b0)
		check_sum_out <= 16'd0;
	else
		check_sum_out <= ~(check_stage[15:0] + check_stage[16]);
end

endmodule
