//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_frame_txd
// 【功能描述】 以太网帧发送模块（增加前导码 + CRC32校验 + BRAM读缓存）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.STATE_PREAMBLE状态负责添加前导码
// 【说明】 3.STATE_DATAPACK状态负责读bram打2拍边校验边发送。
// 【说明】 4.STATE_CHECK状态负责截取CRC32校验结果发送
// 【说明】 5.STATE_DONE状态负责告诉后端发送完成。
// 【说明】 6.
// 【说明】 7.
//
//////////////////////////////////////////////////////////////////////////////////
module eth_frame_txd(
	input				 			etht_clk,					// eth时钟 
	input        			etht_rst,					// eth复位
	output reg [ 7:0] etht_txd_data,		// eth发送数据
	output reg        etht_txd_en,			// eth发送使能
	output reg        etht_txd_done,		// eth发送完成
	input             etht_read_start,	// eth读启动
	input  		 [ 7:0] etht_read_data,		// eth读数据
	input  		 [10:0] etht_read_len,		// eth读长度
	output reg [10:0] etht_read_addr		// eth读地址
);

// 以太网帧发送 状态机定义  
localparam STATE_IDLE  		= 5'b00001;
localparam STATE_PREAMBLE = 5'b00010;
localparam STATE_DATAPACK = 5'b00100;
localparam STATE_CHECK    = 5'b01000;
localparam STATE_DONE 		= 5'b10000;

// 以太网帧发送参数
reg [ 4:0] etht_state;
reg [ 4:0] etht_state_next;
reg [ 2:0] etht_preamble_cnt;
reg [ 7:0] etht_preamble_data;
reg        etht_preamble_done;
reg [ 7:0] etht_datapack_data;
reg [10:0] etht_datapack_cnt;
reg [10:0] etht_datapack_len;
reg        etht_datapack_done;  
reg        etht_read_start_d1;
reg        etht_read_start_d2;
reg [10:0] etht_send_cnt;
reg [10:0] etht_send_len;
reg [ 7:0] etht_check_data;
reg [ 2:0] etht_check_cnt;
reg        etht_check_en;
reg        etht_check_init;
reg        etht_check_done;
reg        etht_frame_crc_en;
reg [ 7:0] etht_read_data_d1;
reg [ 7:0] etht_read_data_d2;
reg        etht_read_en;
wire[31:0] etht_check_buf;

// 寄存器
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_state <= STATE_IDLE;
	else
		etht_state <= etht_state_next;
end

// 状态跳转
always @(*) begin
	case(etht_state)
		STATE_IDLE: begin
			if(etht_read_start_d1)
				etht_state_next = STATE_PREAMBLE;
			else
				etht_state_next = STATE_IDLE;
		end
		STATE_PREAMBLE: begin
			if(etht_preamble_done)
				etht_state_next = STATE_DATAPACK;
			else
				etht_state_next = STATE_PREAMBLE;
		end
		STATE_DATAPACK: begin
			if(etht_datapack_done)
				etht_state_next = STATE_CHECK;
			else
				etht_state_next = STATE_DATAPACK;
		end
		STATE_CHECK: begin
			if(etht_check_done)
				etht_state_next = STATE_DONE;
			else
				etht_state_next = STATE_CHECK;
		end
		STATE_DONE:
			etht_state_next = STATE_IDLE;
		default: etht_state_next = STATE_IDLE;
	endcase
end

// 以太网帧读启动打拍
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0) begin
		etht_read_start_d1 <= 1'b0;
		etht_read_start_d2 <= 1'b0;
	end
	else begin
		etht_read_start_d1 <= etht_read_start;
		etht_read_start_d2 <= etht_read_start_d1;
	end
end

// 以太网真读长度锁存
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_send_len <= 11'd0;
	else if(etht_read_start_d1)
		etht_send_len <= etht_read_len;
end

// 以太网前导码计数
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_preamble_cnt <= 3'd0;
	else if(etht_state == STATE_PREAMBLE)
		etht_preamble_cnt <= etht_preamble_cnt + 1'b1;
	else
		etht_preamble_cnt <= 3'd0;
end

// 添加以太网前导码+帧头
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_preamble_data <= 8'h55;
	else if(etht_preamble_cnt == 3'd6)
		etht_preamble_data <= 8'hd5;
	else
		etht_preamble_data <= 8'h55;
end

// 提前预判以太网前导码计数完成
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_preamble_done <= 1'b0;
	else if(etht_preamble_cnt == 3'd6)
		etht_preamble_done <= 1'b1;
	else
		etht_preamble_done <= 1'b0;
end

// 以太网数据包长度最大计数
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_datapack_len <= 11'd0;
	else if(etht_read_start_d2) begin
		if(etht_send_len < 11'd60)
			etht_datapack_len <= 11'd60;
		else
			etht_datapack_len <= etht_send_len;
	end
end

// 以太网帧数据包计数
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_datapack_cnt <= 11'd0;
	else if(etht_state == STATE_DATAPACK)
		etht_datapack_cnt <= etht_datapack_cnt - 1'b1;
	else
		etht_datapack_cnt <= etht_datapack_len;
end

// 以太网帧数据
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_datapack_data <= 8'd0;
	else if(etht_read_en)
		etht_datapack_data <= etht_read_data_d2;
	else
		etht_datapack_data <= 8'd0;
end

// 以太网帧数据包发送完成不需要填充
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_datapack_done <= 1'b0;
	else if(etht_datapack_cnt == 11'd2)
		etht_datapack_done <= 1'b1;
	else
		etht_datapack_done <= 1'b0;
end

// 以太网校验计数
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_check_cnt <= 3'd0;
	else if(etht_state == STATE_CHECK || etht_datapack_done)
		etht_check_cnt <= etht_check_cnt + 1'b1;
	else
		etht_check_cnt <= 3'd0;
end

// 以太网帧CRC32校验
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_check_data <= 8'd0;
	else begin
		case(etht_check_cnt)
			3'd0: etht_check_data <= etht_check_buf[31:24];
			3'd1: etht_check_data <= etht_check_buf[23:16];
			3'd2: etht_check_data <= etht_check_buf[15: 8];
			3'd3: etht_check_data <= etht_check_buf[ 7: 0];
			default: etht_check_data <= 8'd0;
		endcase
	end
end

// 以太网校验计数完成
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_check_done <= 1'b0;
	else if(etht_check_cnt == 3'd3)
		etht_check_done <= 1'b1;
	else
		etht_check_done <= 1'b0;
end

// 以太网帧发送数据
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_send_cnt <= 11'd0;
	else if(etht_read_start_d2)
		etht_send_cnt <= etht_send_len + 11'd7;
	else if(etht_read_en)
		etht_send_cnt <= etht_send_cnt - 1'b1;
end

// bram读使能
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_read_en <= 1'b0;
	else if(etht_state == STATE_IDLE)
		etht_read_en <= 1'b0;
	else if(etht_state == STATE_PREAMBLE)
		etht_read_en <= 1'b1;
	else if(etht_send_cnt == 11'd1)
		etht_read_en <= 1'b0;
end

// bram读地址
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_read_addr <= 11'd0;
	else if(etht_read_en)
		etht_read_addr <= etht_read_addr + 1'b1;
	else 
		etht_read_addr <= 11'd0;
end

// 2级流水线读无效清零CRC32专用提前计算
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_read_data_d1 <= 8'd0;
	else if(etht_read_en)
		etht_read_data_d1 <= etht_read_data;
	else
		etht_read_data_d1 <= 8'd0;
end

// 3级流水线和发送数据流拼接专用
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0) 
		etht_read_data_d2 <= 8'd0;
	else 
		etht_read_data_d2 <= etht_read_data_d1;
end

// 以太网帧发送数据使能
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_txd_en <= 1'b0;
	else if(etht_state == STATE_IDLE)
		etht_txd_en <= 1'b0;
	else if(etht_state == STATE_PREAMBLE)
		etht_txd_en <= 1'b1;
	else if(etht_state == STATE_DONE)
		etht_txd_en <= 1'b0;
end

// 一级流水线以太网帧发送数据
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_txd_data <= 8'd0;
	else begin
		case(etht_state)
			STATE_PREAMBLE: etht_txd_data <= etht_preamble_data;
			STATE_DATAPACK: etht_txd_data <= etht_datapack_data;
			STATE_CHECK:    etht_txd_data <= etht_check_data;
			default: etht_txd_data <= 8'd0;
		endcase
	end
end

// 以太网帧CRC数据校验使能
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_check_en <= 1'b0;
	else if(etht_preamble_cnt == 3'd5)
		etht_check_en <= 1'b1;
	else if(etht_datapack_cnt == 11'd3)
		etht_check_en <= 1'b0;
end

// 以太网帧CRC数据校验使能
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_check_init <= 1'b0;
	else if(etht_state == STATE_IDLE)
		etht_check_init <= 1'b1;
	else
		etht_check_init <= 1'b0;
end

// 以太网帧数据发送完成
always @(posedge etht_clk or negedge etht_rst) begin
	if(etht_rst == 1'b0)
		etht_txd_done <= 1'd0;
	else if(etht_state == STATE_DONE)
		etht_txd_done <= 1'b1;
	else 
		etht_txd_done <= 1'b0;
end

// 以太网CRC32校验
eth_crc32_8bit eth_crc32_8bit_inst( 
	.crc_clk(etht_clk),				 					// crc校验时钟 
	.crc_rst(etht_rst),				 					// crc校验复位
	.crc_data_in(etht_read_data_d1),		// crc校验数据输入
	.crc_data_en(etht_check_en),				// crc校验数据使能
	.crc_init_val(etht_check_init),  	  // crc校验初始化值
	.crc_data_out(etht_check_buf)   	  // crc校验数据输出
);

endmodule
