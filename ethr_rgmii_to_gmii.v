//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 ethr_rgmii_to_gmii
// 【功能描述】 以太网接收双沿4bit转换单沿8bit
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用直接调用原语，避免创建IP核心繁琐方法。
// 【说明】 2.把数据输入双沿4bit转换单沿8bit数据输出
// 【说明】 3.
// 【说明】 4.
// 【说明】 5.
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
// synopsys translate_off
`timescale 1ps / 1ps
// synopsys translate_on
module altddio_in_x4 (
  input        ddio_clk,
	input  [3:0] ddio_data_in,
	output [3:0] ddio_data_outh,
	output [3:0] ddio_data_outl
);

// altddio ip核
altddio_in	ALTDDIO_IN_component (
				.datain (ddio_data_in),
				.inclock (ddio_clk),
				.dataout_h (ddio_data_outh),
				.dataout_l (ddio_data_outl),
				.aclr (1'b0),
				.aset (1'b0),
				.inclocken (1'b1),
				.sclr (1'b0),
				.sset (1'b0));
	defparam
		ALTDDIO_IN_component.invert_input_clocks = "OFF",
		ALTDDIO_IN_component.lpm_hint = "UNUSED",
		ALTDDIO_IN_component.lpm_type = "altddio_in",
		ALTDDIO_IN_component.power_up_high = "OFF",
		ALTDDIO_IN_component.width = 4;

endmodule
	
module altddio_in_x1 (
  input        ddio_clk,
	input  [0:0] ddio_data_in,
	output [0:0] ddio_data_outh,
	output [0:0] ddio_data_outl
);

// altddio ip核
altddio_in	ALTDDIO_IN_component (
				.datain (ddio_data_in),
				.inclock (ddio_clk),
				.dataout_h (ddio_data_outh),
				.dataout_l (ddio_data_outl),
				.aclr (1'b0),
				.aset (1'b0),
				.inclocken (1'b1),
				.sclr (1'b0),
				.sset (1'b0));
	defparam
		ALTDDIO_IN_component.invert_input_clocks = "OFF",
		ALTDDIO_IN_component.lpm_hint = "UNUSED",
		ALTDDIO_IN_component.lpm_type = "altddio_in",
		ALTDDIO_IN_component.power_up_high = "OFF",
		ALTDDIO_IN_component.width = 1;
endmodule

//
module ethr_rgmii_to_gmii(
	input        ethr_rgmii_clk,			// 以太网rgmii时钟
	input  [3:0] ethr_rgmii_data,			// 以太网rgmii数据
	input        ethr_rgmii_valid,		// 以太网rgmii有效
	output       ethr_gmii_clk,				// 以太网gmii 时钟
	output [7:0] ethr_gmii_data,			// 以太网gmii 数据
	output       ethr_gmii_valid		  // 以太网gmii 有效
);

assign ethr_gmii_clk = ethr_rgmii_clk;

// 输入4bit输出8bit
altddio_in_x4 altddio_in_x4_inst(
	.ddio_clk(ethr_rgmii_clk),
	.ddio_data_in(ethr_rgmii_data),
	.ddio_data_outh(ethr_gmii_data[7:4]),
	.ddio_data_outl(ethr_gmii_data[3:0])
);

// 输入1bit输出1bit
altddio_in_x1 altddio_in_x1_inst(
	.ddio_clk(ethr_rgmii_clk),
	.ddio_data_in(ethr_rgmii_valid),
	.ddio_data_outh(),
	.ddio_data_outl(ethr_gmii_valid)
);

endmodule
