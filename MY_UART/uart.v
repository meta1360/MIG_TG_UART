/***************************************************************************************************
** fpga_nes/hw/src/cmn/uart/uart.v
*
*  Copyright (c) 2012, Brian Bennett
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification, are permitted
*  provided that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this list of conditions
*     and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright notice, this list of
*     conditions and the following disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
*  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
*  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*  UART controller.  Universal Asynchronous Receiver/Transmitter control module for an RS-232
*  (serial) port.
***************************************************************************************************/
/*
`include "uart_baud_clk.v"
`include "uart_rx.v"
`include "uart_tx.v"

`include "fifo.v"
*/
//*****************************************************************
//META
// This uart is personalized for transmission NOT receiving the data 
// we do not care about how receving part is working 
//META END 
//******************************************************************
module uart
#(
  parameter SYS_CLK_FREQ 	 = 50000000,
  parameter BAUD_RATE    	 = 19200,
  parameter DATA_BITS    	 = 96,
  parameter DATA_BITS_INSIDE = 8,
  parameter STOP_BITS     	 = 1,
  parameter PARITY_MODE  	 = 0  // 0 = none, 1 = odd, 2 = even
)
(
  input  wire                 clk,        // System clk
  input  wire                 reset,      // Reset signal
  input  wire                 rx,         // RS-232 rx pin
  input  wire [DATA_BITS-1:0] tx_data,    // Data to be transmitted when wr_en is 1  // this is important to us 
  input  wire                 rd_en,      // Pops current read FIFO front off the queue
  input  wire                 wr_en,      // Write tx_data over serial connection
  output wire                 tx,         // RS-232 tx pin
  output wire [DATA_BITS_INSIDE-1:0] rx_data,    // Data currently at front of read FIFO
  output wire                 tx_empty,   // 1 if there is no more read data available
  output wire                 tx_full,    // 1 if the transmit FIFO cannot accept more requests
  output wire                 parity_err  // 1 if a parity error has been detected
);

localparam BAUD_CLK_OVERSAMPLE_RATE = 16;

wire                 baud_clk_tick;
//META 
wire [DATA_BITS_INSIDE-1:0] rx_fifo_wr_data;
//END META
wire                 rx_done_tick;
wire                 rx_parity_err;
//META
wire [DATA_BITS_INSIDE-1:0] tx_fifo_rd_data;
//END META 
wire                 tx_done_tick;
wire                 tx_fifo_empty;

// Store parity error in a flip flop as persistent state.
reg  q_rx_parity_err;
wire d_rx_parity_err;

always @(posedge clk, posedge reset)
  begin
    if (reset)
      q_rx_parity_err <= 1'b0;
    else
      q_rx_parity_err <= d_rx_parity_err;
  end

assign parity_err      = q_rx_parity_err;
assign d_rx_parity_err = q_rx_parity_err || rx_parity_err;

// BAUD clock module
uart_baud_clk #(.SYS_CLK_FREQ(SYS_CLK_FREQ),
                .BAUD(BAUD_RATE),
                .BAUD_CLK_OVERSAMPLE_RATE(BAUD_CLK_OVERSAMPLE_RATE)) uart_baud_clk_blk
(
  .clk(clk),
  .reset(reset),
  .baud_clk_tick(baud_clk_tick)
);

// RX (receiver) module
uart_rx #(.DATA_BITS(DATA_BITS),
          .STOP_BITS(STOP_BITS),
          .PARITY_MODE(PARITY_MODE),
          .BAUD_CLK_OVERSAMPLE_RATE(BAUD_CLK_OVERSAMPLE_RATE)) uart_rx_blk
(
  .clk(clk),
  .reset(reset),
  .baud_clk_tick(baud_clk_tick),
  .rx(rx),
  .rx_data(rx_fifo_wr_data),
  .rx_done_tick(rx_done_tick),
  .parity_err(rx_parity_err)
);

// TX (transmitter) module
uart_tx #(.DATA_BITS(DATA_BITS_INSIDE),
          .STOP_BITS(STOP_BITS),
          .PARITY_MODE(PARITY_MODE),
          .BAUD_CLK_OVERSAMPLE_RATE(BAUD_CLK_OVERSAMPLE_RATE)) uart_tx_blk
(
  .clk(clk),
  .reset(reset),
  .baud_clk_tick(baud_clk_tick),
  .tx_start(~tx_fifo_empty),
  .tx_data(tx_fifo_rd_data),
  .tx_done_tick(tx_done_tick),
  .tx(tx)
);

// RX FIFO
fifo #(.DATA_BITS(DATA_BITS_INSIDE),
       .size(12),
       .ADDR_BITS(4)) uart_rx_fifo
(
  .clk(clk),
  .reset(reset),
  .rd_en(rd_en),
  .wr_en(rx_done_tick),
  .wr_data(rx_fifo_wr_data),
  .rd_data(rx_data),
  .empty(),
  .full()
);

//META
//***********************************************************
// this part is getting tx_data[544:0] and by every clock pulse
// it send 8 bit portion of data into the input of TX FIFO 
//  ********************************************************** 
wire [7:0] tx_data_8;
wire [3:0] d_wr;
reg [7:0] tx_data_8_r;
reg [3:0] q_wr;

always @(posedge clk)
begin
	if (reset) 
	begin
		tx_data_8_r = 8'b0;
		q_wr = 4'b0;
	end
	else
	begin
		q_wr = d_wr;
		case (q_wr)
		0 : tx_data_8_r = tx_data[7:0]; 
        1 : tx_data_8_r = tx_data[15:8]; 
        2 : tx_data_8_r = tx_data[23:16]; 
        3 : tx_data_8_r = tx_data[31:24]; 
        4 : tx_data_8_r = tx_data[39:32]; 
        5 : tx_data_8_r = tx_data[47:40]; 
        6 : tx_data_8_r = tx_data[55:48]; 
        7 : tx_data_8_r = tx_data[63:56]; 
        8 : tx_data_8_r = tx_data[71:64]; 
        9 : tx_data_8_r = tx_data[79:72]; 
        10 : tx_data_8_r = tx_data[87:80]; 
        11 : tx_data_8_r = tx_data[95:88]; 
   /*     12 : tx_data_8_r = tx_data[103:96]; 
        13 : tx_data_8_r = tx_data[111:104]; 
        14 : tx_data_8_r = tx_data[119:112]; 
        15 : tx_data_8_r = tx_data[127:120]; 
        16 : tx_data_8_r = tx_data[135:128]; 
        17 : tx_data_8_r = tx_data[143:136]; 
        18 : tx_data_8_r = tx_data[151:144]; 
        19 : tx_data_8_r = tx_data[159:152]; 
        20 : tx_data_8_r = tx_data[167:160]; 
        21 : tx_data_8_r = tx_data[175:168]; 
        22 : tx_data_8_r = tx_data[183:176]; 
        23 : tx_data_8_r = tx_data[191:184]; 
        24 : tx_data_8_r = tx_data[199:192]; 
        25 : tx_data_8_r = tx_data[207:200]; 
        26 : tx_data_8_r = tx_data[215:208]; 
        27 : tx_data_8_r = tx_data[223:216]; 
        28 : tx_data_8_r = tx_data[231:224]; 
        29 : tx_data_8_r = tx_data[239:232]; 
        30 : tx_data_8_r = tx_data[247:240]; 
        31 : tx_data_8_r = tx_data[255:248]; 
        32 : tx_data_8_r = tx_data[263:256]; 
        33 : tx_data_8_r = tx_data[271:264]; 
        34 : tx_data_8_r = tx_data[279:272]; 
        35 : tx_data_8_r = tx_data[287:280]; 
        36 : tx_data_8_r = tx_data[295:288]; 
        37 : tx_data_8_r = tx_data[303:296]; 
        38 : tx_data_8_r = tx_data[311:304]; 
        39 : tx_data_8_r = tx_data[319:312]; 
        40 : tx_data_8_r = tx_data[327:320]; 
        41 : tx_data_8_r = tx_data[335:328]; 
        42 : tx_data_8_r = tx_data[343:336]; 
        43 : tx_data_8_r = tx_data[351:344]; 
        44 : tx_data_8_r = tx_data[359:352]; 
        45 : tx_data_8_r = tx_data[367:360]; 
        46 : tx_data_8_r = tx_data[375:368]; 
        47 : tx_data_8_r = tx_data[383:376]; 
        48 : tx_data_8_r = tx_data[391:384]; 
        49 : tx_data_8_r = tx_data[399:392]; 
        50 : tx_data_8_r = tx_data[407:400]; 
        51 : tx_data_8_r = tx_data[415:408]; 
        52 : tx_data_8_r = tx_data[423:416]; 
        53 : tx_data_8_r = tx_data[431:424]; 
        54 : tx_data_8_r = tx_data[439:432]; 
        55 : tx_data_8_r = tx_data[447:440]; 
        56 : tx_data_8_r = tx_data[455:448]; 
        57 : tx_data_8_r = tx_data[463:456]; 
        58 : tx_data_8_r = tx_data[471:464]; 
        59 : tx_data_8_r = tx_data[479:472]; 
        60 : tx_data_8_r = tx_data[487:480]; 
        61 : tx_data_8_r = tx_data[495:488]; 
        62 : tx_data_8_r = tx_data[503:496]; 
        63 : tx_data_8_r = tx_data[511:504]; 
        64 : tx_data_8_r = tx_data[519:512]; 
        65 : tx_data_8_r = tx_data[527:520]; 
        66 : tx_data_8_r = tx_data[535:528]; 
        67 : tx_data_8_r = tx_data[543:536]; */
        default:  tx_data_8_r = tx_data[7:0];
		endcase 
	end
end

		
//assign d_wr = (wr_en) ? (q_wr == 11) ? 4'b0 : q_wr + 1'b1 :
//			  (q_wr == 11) ? q_wr : q_wr + 1'b1; 

assign d_wr = (wr_en && (q_wr == 0)) ? q_wr + 1'b1 : 
									   (q_wr == 0) ? q_wr :
									   ( q_wr == 11 ) ? 0 : q_wr + 1'b1 ;
wire wr_en_inner_tx;
assign wr_en_inner_tx = (wr_en && (q_wr == 0)) ? 1'b1 : 
                        (q_wr == 0) ? 1'b0 : 1'b1;			  

assign tx_data_8 = tx_data_8_r;
//END META


// TX FIFO
fifo #(.DATA_BITS(DATA_BITS_INSIDE),
       .size(12),
       .ADDR_BITS(4)) uart_tx_fifo
(
  .clk(clk),
  .reset(reset),
  .rd_en(tx_done_tick),
  .wr_en(wr_en_inner_tx),
  .wr_data(tx_data_8),
  .rd_data(tx_fifo_rd_data),
  .empty(tx_fifo_empty),
  .full(tx_full)
);

//META
assign tx_empty = tx_fifo_empty; 
//END META
endmodule

