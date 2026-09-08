//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_crc32_8bit
// 【功能描述】 以太网协议栈校验CRC32
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.
// 【说明】 2.
// 【说明】 3.
// 【说明】 4.
// 【说明】 5
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
module eth_crc32_8bit( 
	input				 	crc_clk,				 // crc校验时钟 
	input        	crc_rst,				 // crc校验复位
	input  [ 7:0] crc_data_in,		 // crc校验数据输入
	input         crc_data_en,		 // crc校验数据使能
	input         crc_init_val,    // crc校验初始化值
	output [31:0] crc_data_out     // crc校验数据输出
);

//
reg  [31:0] crc_check_out;
wire [ 7:0] crc_check_in;

// CRC校验输入颠倒0-7bit
assign crc_check_in = {crc_data_in[0], crc_data_in[1], crc_data_in[2], crc_data_in[3],
                       crc_data_in[4], crc_data_in[5], crc_data_in[6], crc_data_in[7]};
// 接收专用CRC校验输出颠倒4字节0-7bit

assign crc_data_out = ~{crc_check_out[24], crc_check_out[25], crc_check_out[26], crc_check_out[27],
											  crc_check_out[28], crc_check_out[29], crc_check_out[30], crc_check_out[31],
											  crc_check_out[16], crc_check_out[17], crc_check_out[18], crc_check_out[19],
											  crc_check_out[20], crc_check_out[21], crc_check_out[22], crc_check_out[23],
											  crc_check_out[ 8], crc_check_out[ 9], crc_check_out[10], crc_check_out[11],
											  crc_check_out[12], crc_check_out[13], crc_check_out[14], crc_check_out[15],
											  crc_check_out[ 0], crc_check_out[ 1], crc_check_out[ 2], crc_check_out[ 3],
											  crc_check_out[ 4], crc_check_out[ 5], crc_check_out[ 6], crc_check_out[ 7]};
											 					 
// 初始CRC校验值
always @(posedge crc_clk or negedge crc_rst) begin
	if(crc_rst == 1'b0)
		crc_check_out <= 32'hFFFFFFFF;
	else if(crc_init_val)
		crc_check_out <= 32'hFFFFFFFF;
	else if(crc_data_en)
		crc_check_out <= calculate_crc32_8bit(crc_check_in, crc_check_out);
end

function [31:0] calculate_crc32_8bit;
	input [7:0] d;      // 输入数据（原始字节，不翻转）
  input [31:0] c;    // 当前CRC值
	reg [31:0] check_next;
	begin
		check_next[0]  = d[6] ^ d[ 0] ^ c[24] ^ c[30];
		check_next[1]  = d[7] ^ d[ 6] ^ d[ 1] ^ d[ 0] ^ c[24] ^ c[25] ^ c[30] ^ c[31];
		check_next[2]  = d[7] ^ d[ 6] ^ d[ 2] ^ d[ 1] ^ d[ 0] ^ c[24] ^ c[25] ^ c[26] ^ c[30] ^ c[31];
		check_next[3]  = d[7] ^ d[ 3] ^ d[ 2] ^ d[ 1] ^ c[25] ^ c[26] ^ c[27] ^ c[31];
		check_next[4]  = d[6] ^ d[ 4] ^ d[ 3] ^ d[ 2] ^ d[ 0] ^ c[24] ^ c[26] ^ c[27] ^ c[28] ^ c[30];
		check_next[5]  = d[7] ^ d[ 6] ^ d[ 5] ^ d[ 4] ^ d[ 3] ^ d[ 1] ^ d[ 0] ^ c[24] ^ c[25] ^ c[27] ^ c[28] ^ c[29] ^ c[30] ^ c[31];
		check_next[6]  = d[7] ^ d[ 6] ^ d[ 5] ^ d[ 4] ^ d[ 2] ^ d[ 1] ^ c[25] ^ c[26] ^ c[28] ^ c[29] ^ c[30] ^ c[31];
		check_next[7]  = d[7] ^ d[ 5] ^ d[ 3] ^ d[ 2] ^ d[ 0] ^ c[24] ^ c[26] ^ c[27] ^ c[29] ^ c[31];
		check_next[8]  = d[4] ^ d[ 3] ^ d[ 1] ^ d[ 0] ^ c[ 0] ^ c[24] ^ c[25] ^ c[27] ^ c[28];
		check_next[9]  = d[5] ^ d[ 4] ^ d[ 2] ^ d[ 1] ^ c[ 1] ^ c[25] ^ c[26] ^ c[28] ^ c[29];
		check_next[10] = d[5] ^ d[ 3] ^ d[ 2] ^ d[ 0] ^ c[ 2] ^ c[24] ^ c[26] ^ c[27] ^ c[29];
		check_next[11] = d[4] ^ d[ 3] ^ d[ 1] ^ d[ 0] ^ c[ 3] ^ c[24] ^ c[25] ^ c[27] ^ c[28];
		check_next[12] = d[6] ^ d[ 5] ^ d[ 4] ^ d[ 2] ^ d[ 1] ^ d[ 0] ^ c[ 4] ^ c[24] ^ c[25] ^ c[26] ^ c[28] ^ c[29] ^ c[30];
		check_next[13] = d[7] ^ d[ 6] ^ d[ 5] ^ d[ 3] ^ d[ 2] ^ d[ 1] ^ c[ 5] ^ c[25] ^ c[26] ^ c[27] ^ c[29] ^ c[30] ^ c[31];
		check_next[14] = d[7] ^ d[ 6] ^ d[ 4] ^ d[ 3] ^ d[ 2] ^ c[ 6] ^ c[26] ^ c[27] ^ c[28] ^ c[30] ^ c[31];
		check_next[15] = d[7] ^ d[ 5] ^ d[ 4] ^ d[ 3] ^ c[ 7] ^ c[27] ^ c[28] ^ c[29] ^ c[31];
		check_next[16] = d[5] ^ d[ 4] ^ d[ 0] ^ c[ 8] ^ c[24] ^ c[28] ^ c[29];
		check_next[17] = d[6] ^ d[ 5] ^ d[ 1] ^ c[ 9] ^ c[25] ^ c[29] ^ c[30];
		check_next[18] = d[7] ^ d[ 6] ^ d[ 2] ^ c[10] ^ c[26] ^ c[30] ^ c[31];
		check_next[19] = d[7] ^ d[ 3] ^ c[11] ^ c[27] ^ c[31];
		check_next[20] = d[4] ^ c[12] ^ c[28];
		check_next[21] = d[5] ^ c[13] ^ c[29];
		check_next[22] = d[0] ^ c[14] ^ c[24];
		check_next[23] = d[6] ^ d[ 1] ^ d[ 0] ^ c[15] ^ c[24] ^ c[25] ^ c[30];
		check_next[24] = d[7] ^ d[ 2] ^ d[ 1] ^ c[16] ^ c[25] ^ c[26] ^ c[31];
		check_next[25] = d[3] ^ d[ 2] ^ c[17] ^ c[26] ^ c[27];
		check_next[26] = d[6] ^ d[ 4] ^ d[ 3] ^ d[ 0] ^ c[18] ^ c[24] ^ c[27] ^ c[28] ^ c[30];
		check_next[27] = d[7] ^ d[ 5] ^ d[ 4] ^ d[ 1] ^ c[19] ^ c[25] ^ c[28] ^ c[29] ^ c[31];
		check_next[28] = d[6] ^ d[ 5] ^ d[ 2] ^ c[20] ^ c[26] ^ c[29] ^ c[30];
		check_next[29] = d[7] ^ d[ 6] ^ d[ 3] ^ c[21] ^ c[27] ^ c[30] ^ c[31];
		check_next[30] = d[7] ^ d[ 4] ^ c[22] ^ c[28] ^ c[31];
		check_next[31] = d[5] ^ c[23] ^ c[29];
		calculate_crc32_8bit = check_next;
	end	
	endfunction
//

endmodule
