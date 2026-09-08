//////////////////////////////////////////////////////////////////////////////////
// 【模块名称】 eth_frame_rxd
// 【功能描述】 以太网帧接收模块（前导码检测 + CRC32校验 + BRAM写缓存）
// 【许可证书】 MIT License
// 【仓库链接】 https://github.com/jakezhao2018-spec/fpga_ethernet_stack
// 【芯片型号】 EP4CE10F17C8
// 【实验环境】 AC620开发板
// 【作者】 jakezhao
// 【日期】 2026-09-03
// 【版本】 v1.0
// 【说明】 1.采用三段式状态机结构清晰。
// 【说明】 2.帧有效跳转到前导码解析
// 【说明】 3.前导码解析正确跳转到数据包解析，否则跳转到DONE。
// 【说明】 4.流水线打5拍数据边写BRAM，边计算CRC32
// 【说明】 5.帧异常在DONE模式等待当前帧完成退出
// 【说明】 6.最后一个数据写BRAM完成，CRC计算也完成。
// 【说明】 7.校验CRC一致输出done告诉后端接收到帧的数据帧可以解析
//
//////////////////////////////////////////////////////////////////////////////////
module eth_frame_rxd(
	input				 			ethr_clk,					// eth时钟 
	input        			ethr_rst,					// eth复位
	input      [ 7:0] ethr_rxd_data,		// eth接收数据
	input             ethr_rxd_valid,		// eth接收有效
	output reg [ 7:0] ethr_write_data,	// eth写数据
	output reg [10:0] ethr_write_len,		// eth写长度
	output reg [10:0] ethr_write_addr,	// eth写计数
	output reg        ethr_write_en,		// eth写使能
	output reg   			ethr_write_done		// eth写完成
);

// 以太网帧接收 状态机定义  
localparam STATE_IDLE  		= 5'b00001;
localparam STATE_PREAMBLE = 5'b00010;
localparam STATE_DATAPACK = 5'b00100;
localparam STATE_CHECK    = 5'b01000;
localparam STATE_DONE 		= 5'b10000;

// 以太网帧接收参数
reg [ 4:0] ethr_state;
reg [ 4:0] ethr_state_next;
reg [ 7:0] ethr_rxd_data_d1;
reg [ 7:0] ethr_rxd_data_d2;
reg [ 7:0] ethr_rxd_data_d3;
reg [ 7:0] ethr_rxd_data_d4;
reg [ 7:0] ethr_rxd_data_d5;
reg        ethr_rxd_valid_d1;
reg        ethr_rxd_valid_d2;
reg [ 2:0] ethr_preamble_cnt;
reg        ethr_preamble_done;
reg        ethr_preamble_error;
reg        ethr_datapack_done;
reg [31:0] ethr_check_crc;
reg        ethr_check_en;
reg        ethr_check_init;
reg        ethr_write_en_d1;
reg        ethr_frame_done;
reg [31:0] ethr_frame_crc;
wire[31:0] ethr_check_buf;

// 寄存器
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_state <= STATE_IDLE;
	else
		ethr_state <= ethr_state_next;
end

// 状态跳转
always @(*) begin
	case(ethr_state)
		STATE_IDLE: begin
			if(ethr_rxd_valid_d1)
				ethr_state_next = STATE_PREAMBLE;
			else
				ethr_state_next = STATE_IDLE;
		end
		STATE_PREAMBLE: begin
			if(ethr_preamble_error)
				ethr_state_next = STATE_DONE;
			else if(ethr_preamble_done)
				ethr_state_next = STATE_DATAPACK;
			else
				ethr_state_next = STATE_PREAMBLE;
		end
		STATE_DATAPACK: begin
			if(ethr_datapack_done)
				ethr_state_next = STATE_CHECK;
			else
				ethr_state_next = STATE_DATAPACK;
		end
		STATE_CHECK: begin
			ethr_state_next = STATE_DONE;
		end
		STATE_DONE: begin
			if(ethr_frame_done)
				ethr_state_next = STATE_IDLE;
			else
				ethr_state_next = STATE_DONE;
		end
		default: ethr_state_next = STATE_IDLE;
	endcase
end

// 接收使能打拍
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0) begin
		ethr_rxd_valid_d1 <= 1'b0;
		ethr_rxd_valid_d2 <= 1'b0;
	end
	else begin
		ethr_rxd_valid_d1 <= ethr_rxd_valid;
		ethr_rxd_valid_d2 <= ethr_rxd_valid_d1;
	end
end

// 接收数据缓存4拍
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0) begin
		ethr_rxd_data_d1 <= 8'd0;
		ethr_rxd_data_d2 <= 8'd0;
		ethr_rxd_data_d3 <= 8'd0;
		ethr_rxd_data_d4 <= 8'd0;
		ethr_rxd_data_d5 <= 8'd0;
	end 
	else begin
		ethr_rxd_data_d1 <= ethr_rxd_data;
		ethr_rxd_data_d2 <= ethr_rxd_data_d1;
		ethr_rxd_data_d3 <= ethr_rxd_data_d2;
		ethr_rxd_data_d4 <= ethr_rxd_data_d3;
		ethr_rxd_data_d5 <= ethr_rxd_data_d4;
	end
end

// 以太网前导码55连续计数
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_preamble_cnt <= 3'd0;
	else if(ethr_state == STATE_PREAMBLE && ethr_rxd_data_d3 == 8'h55)
		ethr_preamble_cnt <= ethr_preamble_cnt + 1'b1;
	else
		ethr_preamble_cnt <= 3'd0;
end

// 识别以太网开始帧和前导码SFD_5D有效
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_preamble_done <= 1'b0;
	else if(ethr_rxd_data_d3 == 8'hd5 && ethr_preamble_cnt >= 3'd3)
		ethr_preamble_done <= 1'b1;
	else
		ethr_preamble_done <= 1'b0;
end

// 以太网前导码错误标识
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_preamble_error <= 1'b0;
	else if(ethr_state == STATE_PREAMBLE) begin
		if(ethr_rxd_data_d3 == 8'hd5 && ethr_preamble_cnt < 3'd3)
			ethr_preamble_error <= 1'b1;
		else
			ethr_preamble_error <= ~ethr_rxd_valid_d1;
	end
	else
		ethr_preamble_error <= 1'b0;
end

// bram写使能
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_write_en <= 1'b0;
	else if(ethr_state == STATE_IDLE)
		ethr_write_en <= 1'b0;
	else if(ethr_state == STATE_DATAPACK)
		ethr_write_en <= ethr_rxd_valid_d1;
end

// bram写使能打拍
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_write_en_d1 <= 1'b0;
	else
		ethr_write_en_d1 <= ethr_write_en;
end

// bram写数据
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_write_data <= 8'd0;
	else if(ethr_state == STATE_DATAPACK)
		ethr_write_data <= ethr_rxd_data_d4;
end

// bram写地址偏移计数
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_write_addr <= 11'd0;
	else if(ethr_write_en)
		ethr_write_addr <= ethr_write_addr + 1'b1;
	else 
		ethr_write_addr <= 11'd0;
end

// 锁存bram写入完整以太网帧数据大小
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_write_len <= 11'd0;
	else if(ethr_rxd_valid_d2 && !ethr_rxd_valid_d1)
		ethr_write_len <= ethr_write_addr;
end

// 以太网帧数据接收完成
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_datapack_done <= 1'b0;
	else if(ethr_state == STATE_DATAPACK)
		ethr_datapack_done <= ~ethr_rxd_valid_d1;
	else
		ethr_datapack_done <= 1'b0;
end

// 接收提取以太网帧CRC校验
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_frame_crc <= 32'd0;
	else if(ethr_rxd_valid_d2 && !ethr_rxd_valid_d1)
		ethr_frame_crc <= {ethr_rxd_data_d5, ethr_rxd_data_d4, ethr_rxd_data_d3, ethr_rxd_data_d2};
end


// 以太网帧校验初始化
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_check_init <= 1'b0;
	else if(ethr_state == STATE_IDLE)
		ethr_check_init <= 1'b1;
	else
		ethr_check_init <= 1'b0;
end

// 以太网帧校验使能
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_check_en <= 1'b0;
	else if(ethr_state == STATE_DATAPACK)
		ethr_check_en <= ethr_rxd_valid_d1;
	else 
		ethr_check_en <= 1'b0;
end

// 以太网帧数据校验CRC锁存
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_check_crc <= 32'd0;
	else if(ethr_rxd_valid_d2 && !ethr_rxd_valid_d1)
		ethr_check_crc <= ethr_check_buf;
end

// bram写入一个完整的以太网帧完成
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_write_done <= 1'b0;
	else if(ethr_state == STATE_CHECK)
		ethr_write_done <= ethr_frame_crc == ethr_check_crc;
	else 
		ethr_write_done <= 1'b0;
end

// 等待以太网帧接收完成
always @(posedge ethr_clk or negedge ethr_rst) begin
	if(ethr_rst == 1'b0)
		ethr_frame_done <= 1'b0;
	else if(ethr_state == STATE_DONE)
		ethr_frame_done <= ~ethr_rxd_valid_d1;
	else
		ethr_frame_done <= 1'b0;
end

// 以太网CRC32校验
eth_crc32_8bit eth_crc32_8bit_inst( 
	.crc_clk(ethr_clk),				 				// crc校验时钟 
	.crc_rst(ethr_rst),				 				// crc校验复位
	.crc_data_in(ethr_rxd_data_d5),		// crc校验数据输入
	.crc_data_en(ethr_check_en),		 	// crc校验数据使能
	.crc_init_val(ethr_check_init),   // crc校验初始化值
	.crc_data_out(ethr_check_buf)     // crc校验数据输出  							
);

endmodule
