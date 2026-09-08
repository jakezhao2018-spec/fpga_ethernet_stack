//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 cdc_pulse_toggle
// 【功能描述】 cdc脉冲触发模块
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.解决跨时钟域脉冲同步
// 【说明】 2.
// 【说明】 3.
// 【说明】 4.
// 【说明】 5.
// 【说明】 6.
//
//////////////////////////////////////////////////////////////////////////////////
module cdc_pulse_toggle(
	input			 src_clk,			// 源时钟
	input   	 src_rst,			// 源复位
	input      src_pulse,		// 源脉冲
	input      dst_clk,			// 目的时钟
	input      dst_rst,			// 目的复位
	output     dst_pulse		// 目的脉冲
);

//
reg signal_toggle;
(* ASYNC_REG = "TRUE" *)reg [2:0] signal_sync;

// 
assign dst_pulse = signal_sync[2] ^ signal_sync[1];

// 源脉冲输入触发检测
always @(posedge src_clk or negedge src_rst) begin
	if(src_rst == 0) 
		signal_toggle <= 1'b0;
	else if(src_pulse)
		signal_toggle <= ~signal_toggle;
end

// 源脉冲触发信号同步
always @(posedge dst_clk or negedge dst_rst) begin
	if(dst_rst == 0)
		signal_sync <= 3'b0;
	else 
		signal_sync <= {signal_sync[1:0], signal_toggle};
end

endmodule
