//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 etht_gmii_to_rgmii
// 【功能描述】 以太网发送单沿8bit转换双沿4bit
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用直接调用原语，避免创建IP核心繁琐方法。
// 【说明】 2.把数据输入单沿8bit转换双沿4bit数据输出
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

// 8bit转4bit
module altddio_out_x4 (
  input        ddio_clk,
	input  [3:0] ddio_data_inh,
	input  [3:0] ddio_data_inl,
	output [3:0] ddio_data_out
);

// altddio ip核
altddio_out	ALTDDIO_OUT_component (
				.datain_h (ddio_data_inh),
				.datain_l (ddio_data_inl),
				.dataout  (ddio_data_out),
				.outclock (ddio_clk),
				.aclr (1'b0),
				.aset (1'b0),
				.oe(1'b1),
				.oe_out(),
				.outclocken (1'b1),
				.sclr (1'b0),
				.sset (1'b0));
	defparam
		ALTDDIO_OUT_component.extend_oe_disable = "OFF",
		ALTDDIO_OUT_component.invert_output = "OFF",
		ALTDDIO_OUT_component.lpm_hint = "UNUSED",
		ALTDDIO_OUT_component.lpm_type = "altddio_out",
		ALTDDIO_OUT_component.oe_reg = "UNREGISTERED",
		ALTDDIO_OUT_component.power_up_high = "OFF",
		ALTDDIO_OUT_component.width = 4;
endmodule

// 1bit转1bit
module altddio_out_x1 (
  input        ddio_clk,
	input  [0:0] ddio_data_inh,
	input  [0:0] ddio_data_inl,
	output [0:0] ddio_data_out
);

// altddio ip核
altddio_out	ALTDDIO_OUT_component (
				.datain_h (ddio_data_inh),
				.datain_l (ddio_data_inl),
				.dataout (ddio_data_out),
				.outclock (ddio_clk),
				.aclr (1'b0),
				.aset (1'b0),
				.oe(1'b1),
				.oe_out(),
				.outclocken (1'b1),
				.sclr (1'b0),
				.sset (1'b0));
	defparam
		ALTDDIO_OUT_component.extend_oe_disable = "OFF",
		ALTDDIO_OUT_component.invert_output = "OFF",
		ALTDDIO_OUT_component.lpm_hint = "UNUSED",
		ALTDDIO_OUT_component.lpm_type = "altddio_out",
		ALTDDIO_OUT_component.oe_reg = "UNREGISTERED",
		ALTDDIO_OUT_component.power_up_high = "OFF",
		ALTDDIO_OUT_component.width = 1;
endmodule

// 以太网发送gmii转rgmii
module etht_gmii_to_rgmii(
	input        etht_gmii_clk,			// 以太网发送rgmii时钟	
	input  [7:0] etht_gmii_data,		// 以太网发送rgmii数据
	input        etht_gmii_en,			// 以太网发送rgmii使能
	output       etht_rgmii_clk,		// 以太网发送gmii 时钟
	output [3:0] etht_rgmii_data,		// 以太网发送gmii 数据
	output       etht_rgmii_en		 	// 以太网发送gmii 使能
);

// 输入4bit输出8bit
altddio_out_x4 altddio_out_x4_inst(
	.ddio_clk(etht_gmii_clk),
	.ddio_data_inh(etht_gmii_data[3:0]),
	.ddio_data_inl(etht_gmii_data[7:4]),
	.ddio_data_out(etht_rgmii_data)
);

// 输入1bit输出1bit
altddio_out_x1 altddio_out_x1_inst(
	.ddio_clk(etht_gmii_clk),
	.ddio_data_inh(etht_gmii_en),
	.ddio_data_inl(etht_gmii_en),
	.ddio_data_out(etht_rgmii_en)
);

// 输入1bit输出1bit
altddio_out_x1 altddio_out_x1_clk(
	.ddio_clk(etht_gmii_clk),
	.ddio_data_inh(1'b1),
	.ddio_data_inl(1'b0),
	.ddio_data_out(etht_rgmii_clk)
);

endmodule
