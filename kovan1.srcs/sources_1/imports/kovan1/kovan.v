////////////////////////////////////////////////
// Copyright (c) 2012, Andrew "bunnie" Huang  
// (bunnie _aht_ bunniestudios "dote" com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//     Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////
  
`timescale 1 ns / 1 ps

parameter C3_NUM_DQ_PINS          = 16;   // External memory data width
parameter C3_MEM_ADDR_WIDTH       = 13;   // External memory address width
parameter C3_MEM_BANKADDR_WIDTH   = 3;    // External memory bank address width

`define HAS_DDR    // comment out to remove DDR interface (does not currently work)

module kovan (
	      // camera IF
	      output wire [7:0] CAM_D,
	      output wire       CAM_HSYNC,  // sync
	      output wire       CAM_VSYNC,  // pix valid / hsync
	      input wire        CAM_MCLKO,  // pixel master clock
	      input wire        CAM_VCLKO,  // pixel clock from CPU
	      output wire       CAM_PCLKI,  // return pixel clock

	      // power management
	      input wire CHG_ACP,           // reports presence of AC power
	      output wire CHG_SHDN,         // pull low to turn board off

	      // HDMI
	      input wire        CEC,
	      input  wire       DDC_SDA_LV_N,
	      output wire       DDC_SDA_PU,
	      output wire       DDC_SDA_PD,
	      input  wire       DDC_SCL_LV_N,
	      input  wire       HPD_N,
	      output wire       HPD_NOTIFY,
	      output wire       HPD_OVERRIDE,
	      output wire       VSYNC_STB,

	      // HDMI high speed phy lines
	      input wire [3:0]  RX0_TMDS_P,
	      input wire [3:0]  RX0_TMDS_N,
	      output wire [3:0] TX0_TMDS_P,
	      output wire [3:0] TX0_TMDS_N,

	      // i/o controller digital interfaces
	      output wire [1:0] DIG_ADC_CS,
	      output wire       DIG_ADC_IN,
	      input wire        DIG_ADC_OUT,
	      output wire       DIG_ADC_SCLK,
	      output wire       DIG_ADC_CLR,
	      output wire       DIG_IN,
	      input wire        DIG_OUT,
	      output wire       DIG_RCLK,
	      output wire       DIG_SAMPLE,
	      output wire       DIG_SCLK,
	      output wire       DIG_SRLOAD,
	      output wire       DIG_CLR_N,

	      // motor direct drive interfaces
	      output wire [3:0] MBOT,
	      output wire [3:0] MTOP,
	      output wire       MOT_EN,
	      output wire [3:0] M_SERVO,

	      // optional uart to outside world
	      input wire        EXT_TO_HOST_UART, // for now we're a fly on the wall
	      input wire        HOST_TO_EXT_UART,

	      // infrared receiver 
	      input wire        IR_RX,

	      // switch
	      input wire        INPUT_SW0,

	      // audio pass-through
	      input wire        I2S_CDCLK0, // master reference clock to audio PLL
	      output wire       I2S_CDCLK1,
	      output wire       I2S_CLK0,   // return sample clock to CPU
	      input wire        I2S_CLK1,
	      input wire        I2S_DI0,    // audio data to playback
	      output wire       I2S_DI1,
	      output wire       I2S_DO0,    // audio data from record
	      input wire        I2S_DO1,
	      output wire       I2S_LRCLK0, // left/right clock to codec
	      input wire        I2S_LRCLK1,

	      // LCD output to display
	      output wire [7:3] LCDO_B,  // note truncation of blue channel
	      output wire [7:2] LCDO_G,
	      output wire [7:2] LCDO_R,
	      output wire       LCDO_DEN,
	      output wire       LCDO_DOTCLK,
	      output wire       LCDO_HSYNC,
	      output wire       LCDO_RESET_N,
	      output wire       LCDO_VSYNC,

	      // LCD input from CPU
	      input wire [5:0]  LCD_B,  // note no truncation of data in
	      input wire [5:0]  LCD_G,
	      input wire [5:0]  LCD_R,
	      input wire        LCD_DEN,
	      input wire        LCD_HS,
	      input wire [5:0]  LCD_SUPP,
	      input wire        LCD_VS,
	      output wire       LCD_CLK_T,  // clock is sourced from the FPGA
	      // for forward compatibility with HDMI-synced streams

	      // SSP interface to the CPU
	      output wire       FPGA_MISO,
	      input wire        FPGA_MOSI,
	      input wire        FPGA_SCLK,
	      input wire        FPGA_SYNC,

	      // I2C interfaces
	      input wire        PWR_SCL,  // we listen on this one
	      inout wire        PWR_SDA,

	      input wire        XI2CSCL,  // our primary interface
	      inout wire        XI2CSDA,

	      // LED
	      output wire       FPGA_LED,

	      // mcb interface to DDR2
	      inout  [C3_NUM_DQ_PINS-1:0]          mcb3_dram_dq,
	      output [C3_MEM_ADDR_WIDTH-1:0]       mcb3_dram_a,
	      output [C3_MEM_BANKADDR_WIDTH-1:0]   mcb3_dram_ba,
	      output                               mcb3_dram_ras_n,
	      output                               mcb3_dram_cas_n,
	      output                               mcb3_dram_we_n,
	      output                               mcb3_dram_odt,
	      output                               mcb3_dram_cke,
	      output                               mcb3_dram_dm,
	      inout                                mcb3_dram_udqs,
	      inout                                mcb3_dram_udqs_n,
	      inout                                mcb3_rzq,
	      inout                                mcb3_zio,
	      output                               mcb3_dram_udm,
	      inout                                mcb3_dram_dqs,
	      inout                                mcb3_dram_dqs_n,
	      output                               mcb3_dram_ck,
	      output                               mcb3_dram_ck_n,

	      input wire       OSC_CLK   // 26 mhz clock from CPU
	      );

   ///////// clock buffers
   wire            clk26;
   wire 	   clk26ibuf;
   wire 	   clk26buf;
   wire 	   clk13buf;
   wire            clk3p2M;
   wire 	   clk208M;
   wire            clk1M;  // wired up in the serial number section
   
   assign clk26 = OSC_CLK;
   IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));
   BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));

   
   ////////// reset
   reg   	   glbl_reset; // to be used sparingly
   wire            glbl_reset_edge;
   reg 		   glbl_reset_edge_d;

   always @(posedge clk1M) begin
      glbl_reset_edge_d <= glbl_reset_edge;
      glbl_reset <= !glbl_reset_edge_d & glbl_reset_edge; // just pulse reset for one cycle of the slowest clock in the system
   end
   
   ////////// loop-throughs
   // lcd runs at a target of 6.41 MHz (156 ns cycle time)
   // i.e., 408 x 262 x 60 Hz (408 is total H width, 320 active, etc.)
   wire            qvga_clkgen_locked;
   
   clk_wiz_v3_2_qvga qvga_clkgen( .CLK_IN1(clk26buf),
				  .clk_out6p4(clk_qvga),
				  .clk_out13(clk13buf),
				  .clk_out3p25(clk3p2M), // note: a slight overclock (about 2%)
				  .clk_out208(clk208M),
				  .RESET(glbl_reset),
				  .LOCKED(qvga_clkgen_locked) );
   
   reg 	[5:0]	   lcd_pipe_b;
   reg 	[5:0]	   lcd_pipe_r;
   reg 	[5:0]	   lcd_pipe_g;
   reg 		   lcd_pipe_den;
   reg 		   lcd_hsync;
   reg 		   lcd_vsync;
   reg 		   lcd_reset_n;
   reg 		   lcd_pipe_hsync;
   reg 		   lcd_pipe_vsync;
   wire 	   lcd_reset;

   sync_reset  qvga_reset(
			  .clk(clk_qvga),
			  .glbl_reset(glbl_reset || !qvga_clkgen_locked),
			  .reset(lcd_reset) );
   always @(posedge clk_qvga) begin
      // TODO: assign timing constraints to ensure hold times met for LCD
      lcd_pipe_b[5:0] <= LCD_B[5:0];
      lcd_pipe_g[5:0] <= LCD_G[5:0];
      lcd_pipe_r[5:0] <= LCD_R[5:0];
      lcd_pipe_den <= LCD_DEN;
      lcd_pipe_hsync <= LCD_HS;
      lcd_pipe_vsync <= LCD_VS;
      lcd_reset_n <= !lcd_reset;
   end

   assign LCDO_B[7:3] = lcd_pipe_b[5:1];
   assign LCDO_G[7:2] = lcd_pipe_g[5:0];
   assign LCDO_R[7:2] = lcd_pipe_r[5:0];
   assign LCDO_DEN = lcd_pipe_den;
   assign LCDO_HSYNC = lcd_pipe_hsync;
   assign LCDO_VSYNC = lcd_pipe_vsync;
   assign LCDO_RESET_N = lcd_reset_n;

   // low-skew clock mirroring to an output pin requires this hack
   ODDR2 qvga_clk_to_lcd (.D0(1'b1), .D1(1'b0), 
			  .C0(clk_qvga), .C1(!clk_qvga), 
			  .Q(LCDO_DOTCLK), .CE(1'b1), .R(1'b0), .S(1'b0) );

   ODDR2 qvga_clk_to_cpu (.D0(1'b1), .D1(1'b0), 
			 .C0(clk_qvga), .C1(!clk_qvga), 
			 .Q(LCD_CLK_T), .CE(1'b1), .R(1'b0), .S(1'b0) );


   ///////////////////////////////////////////
   // audio pass-through -- unbuffurred, unregistered for now
   assign I2S_CDCLK1 = I2S_CDCLK0;
   assign I2S_CLK0 = I2S_CLK1;
   assign I2S_DI1 = I2S_DI0;
   assign I2S_DO0 = I2S_DO1;
   assign I2S_LRCLK0 = I2S_LRCLK1;
   
   ///////////////////////////////////////////
   // motor control unit
   wire [7:0] dig_out_val;
   wire [7:0] dig_oe;
   wire [7:0] dig_pu;
   wire [7:0] ana_pu;
   wire [7:0] dig_in_val;
   wire       dig_val_good;
   wire       dig_busy;
   wire       dig_sample;
   wire       dig_update;

   wire [9:0] adc_in;
   wire [3:0] adc_chan;
   wire       adc_valid;
   wire       adc_go;

   wire [15:0] mot_pwm_div;
   wire [15:0] mot_pwm_duty;
   wire [7:0]  mot_drive_code;
   wire        mot_allstop;

   wire [23:0] servo_pwm_period;
   wire [23:0] servo0_pwm_pulse;
   wire [23:0] servo1_pwm_pulse;
   wire [23:0] servo2_pwm_pulse;
   wire [23:0] servo3_pwm_pulse;

   robot_iface iface(.clk(clk13buf), .glbl_reset(glbl_reset),
		     .clk_3p2MHz(clk3p2M), .clk_208MHz(clk208M),

	     // digital i/o block
	     .dig_out_val(dig_out_val),
	     .dig_oe(dig_oe),
	     .dig_pu(dig_pu),
	     .ana_pu(ana_pu),
	     .dig_in_val(dig_in_val),
	     .dig_val_good(dig_val_good), // output value is valid when high
	     .dig_busy(dig_busy),    // chain is busy when high
	     .dig_sample(dig_sample),  // samples input on rising edge
	     .dig_update(dig_update),  // updates chain on rising edge

	     // ADC interface
	     .adc_in(adc_in),
	     .adc_chan(adc_chan),    // channels 0-7 are for user, 8-15 are for motor current fbk
	     .adc_valid(adc_valid),
	     .adc_go(adc_go),  

	     // motor driver interface
	     .mot_pwm_div(mot_pwm_div),
	     .mot_pwm_duty(mot_pwm_duty),
	     .mot_drive_code(mot_drive_code), // 2 bits/chan, 00 = stop, 01 = forward, 10 = rev, 11 = stop
	     .mot_allstop(mot_allstop),

	     // servo interface
	     .servo_pwm_period(servo_pwm_period), // total period for the servo update
	     .servo0_pwm_pulse(servo0_pwm_pulse), // pulse width in absolute time
	     .servo1_pwm_pulse(servo1_pwm_pulse),
	     .servo2_pwm_pulse(servo2_pwm_pulse),
	     .servo3_pwm_pulse(servo3_pwm_pulse),

	     /////// physical interfaces to outside the chip
	     // motors
	     .MBOT(MBOT[3:0]),
	     .MTOP(MTOP[3:0]),
	     .MOT_EN(MOT_EN),
	     .M_SERVO(M_SERVO[3:0]),

	     // analog interface
	     .DIG_ADC_CS(DIG_ADC_CS),
	     .DIG_ADC_IN(DIG_ADC_IN),
	     .DIG_ADC_OUT(DIG_ADC_OUT),
	     .DIG_ADC_SCLK(DIG_ADC_SCLK),

	     // digital interface
	     .DIG_IN(DIG_IN),
	     .DIG_OUT(DIG_OUT),
	     .DIG_RCLK(DIG_RCLK),
	     .DIG_SAMPLE(DIG_SAMPLE),
	     .DIG_SCLK(DIG_SCLK),
	     .DIG_SRLOAD(DIG_SRLOAD),
	     .DIG_CLR_N(DIG_CLR_N)
		     );
   

`ifdef HAS_DDR   
   ///////////////////////////////////////////
   /////// DDR2 core 128 MB x 16 of memory, 312 MHz (624 Mt/s = 1.2 GiB/s)
   // generate a 312 MHz clock from the 26 MHz local buffer
   wire c3_sys_clk;
   wire c3_clk_locked;
   wire c3_clk_fb_from_clkgen;
   wire c3_clk_fb_to_clkgen;
   clk_ddr2_26m_312m ddr2_clk(
			      .CLK_IN1(clk26buf),
			      .CLKFB_IN(c3_clk_fb_to_clkgen),
			      .CLK_OUT1(c3_sys_clk),
			      .CLKFB_OUT(c3_clk_fb_from_clkgen),
			      .RESET(glbl_reset),
			      .LOCKED(c3_clk_locked)
			      );
   BUFG ddr2_clkf_buf
     (.O (c3_clk_fb_to_clkgen),
      .I (c3_clk_fb_from_clkgen));

   // instantiate the core
   wire c3_clk0; // clock to internal fabric
   wire c3_rst0;
   reg [31:0] ddr2_read_data;
   wire [31:0] ddr2_read_data_c3;
   reg [31:0]  ddr2_read_data_c3_save;
   wire [31:0] ddr2_write_data;
   reg [31:0] ddr2_write_data_c3;
   wire [29:0]  ddr2_test_addr;
   reg [29:0]  ddr2_test_addr_c3;

   // local control logic
   reg 	       ddr2_wren;
   reg 	       ddr2_rden;
   reg 	       ddr2_rden_d;
   reg 	       ddr2_wr_cmd_en;
   reg 	       ddr2_rd_cmd_en;
   wire        ddr2_rd_avail_n;
   wire        ddr2_wr_empty;
   wire        ddr2_reset;

   // register interface controls
   wire        ddr2_dowrite;
   wire        ddr2_doread;
   wire [7:0]  ddr2_regcmd;
   reg [7:0]   ddr2_regcmd_c3;
   reg [7:0]   ddr2_regstat;
   wire [7:0]  ddr2_regstat_c3;

   wire        ddr2_rdwr;
   wire        ddr2_docmd;
   reg 	       ddr2_docmd_d;

   reg [7:0]   ddr2_sm_dbg;
   
   ddr2_m3_core_v2 ddr2core(
			    // external wires
			    .mcb3_dram_dq(mcb3_dram_dq),
			    .mcb3_dram_a(mcb3_dram_a),
			    .mcb3_dram_ba(mcb3_dram_ba),
			    .mcb3_dram_ras_n(mcb3_dram_ras_n),
			    .mcb3_dram_cas_n(mcb3_dram_cas_n),
			    .mcb3_dram_we_n(mcb3_dram_we_n),
			    .mcb3_dram_odt(mcb3_dram_odt),
			    .mcb3_dram_cke(mcb3_dram_cke),
			    .mcb3_dram_dm(mcb3_dram_dm),
			    .mcb3_dram_udqs(mcb3_dram_udqs),
			    .mcb3_dram_udqs_n(mcb3_dram_udqs_n),
			    .mcb3_rzq(mcb3_rzq),
			    .mcb3_zio(mcb3_zio),
			    .mcb3_dram_udm(mcb3_dram_udm),
			    .mcb3_dram_dqs(mcb3_dram_dqs),
			    .mcb3_dram_dqs_n(mcb3_dram_dqs_n),
			    .mcb3_dram_ck(mcb3_dram_ck),
			    .mcb3_dram_ck_n(mcb3_dram_ck_n),

			    // clock and reset
			    .c3_sys_clk(c3_sys_clk), 
			    .c3_sys_rst_i(ddr2_reset),
			    .c3_calib_done(c3_calib_done),
			    .c3_clk0(c3_clk0), 
			    .c3_rst0(c3_rst0), 

			    // internal interfaces. only two brought out here for testing
			    // port 2 - read-only
			    .c3_p2_cmd_clk(c3_clk0),
			    .c3_p2_cmd_en(ddr2_rd_cmd_en),
			    .c3_p2_cmd_instr(3'b011),
			    .c3_p2_cmd_bl(6'b1),
			    .c3_p2_cmd_byte_addr(ddr2_test_addr_c3),
			    .c3_p2_rd_clk(c3_clk0),
			    .c3_p2_rd_en(ddr2_rden),
			    .c3_p2_rd_data(ddr2_read_data_c3),
			    .c3_p2_rd_empty(ddr2_rd_avail_n),

			    // port 3 - write-only
			    .c3_p3_cmd_clk(c3_clk0),
			    .c3_p3_cmd_en(ddr2_wr_cmd_en),
			    .c3_p3_cmd_instr(3'b010),
			    .c3_p3_cmd_bl(6'b1),
			    .c3_p3_cmd_byte_addr(ddr2_test_addr_c3),
			    .c3_p3_wr_clk(c3_clk0),
			    .c3_p3_wr_en(ddr2_wren),
			    .c3_p3_wr_mask(4'hf),
			    .c3_p3_wr_data(ddr2_write_data_c3),
			    .c3_p3_wr_empty(ddr2_wr_empty)
			    );

   // datapath wiring

   // retime register interface data to c3 clock domain
   always @(posedge c3_clk0) begin
      ddr2_write_data_c3 <= ddr2_write_data;
      ddr2_test_addr_c3 <= ddr2_test_addr;
      ddr2_regcmd_c3 <= ddr2_regcmd;

      ddr2_docmd_d = ddr2_docmd; // delayed version for rising edge detect
      ddr2_rden_d <= ddr2_rden;
      // lock in the data only when we've got it
      // theory is data is available one clock after rden
      // timing is unclear though
      if( ddr2_rden_d ) begin
	 ddr2_read_data_c3_save <= ddr2_read_data_c3;
      end else begin
	 ddr2_read_data_c3_save <= ddr2_read_data_c3_save;
      end
   end

   // retime c3 clock domain data into the register interface domain
   always @(posedge clk26buf) begin
      ddr2_read_data <= ddr2_read_data_c3_save;
      ddr2_regstat <= ddr2_regstat_c3;

      // debug the state machine (only effective to see if we're wedged, most transitions too fast to catch via I2C)
      ddr2_sm_dbg[7:4] <= DDR2_WR_cstate[3:0];
      ddr2_sm_dbg[3:0] <= DDR2_RD_cstate[3:0];
   end

   // local reset timer for ddr2
   sync_reset  ddr2_reset_sync(
			  .clk(c3_clk0),
			  .glbl_reset(!c3_clk_locked),
			  .reset(ddr2_reset) );
   
   ////// command and control mappings
   assign ddr2_regstat_c3[0] = !ddr2_rd_avail_n;
   assign ddr2_regstat_c3[1] = ddr2_wr_empty;
   assign ddr2_regstat_c3[2] = c3_calib_done;
   assign ddr2_rdwr = ddr2_regcmd_c3[0]; // 1 is read, 0 is write
   assign ddr2_docmd = ddr2_regcmd_c3[1];

   ///////////
   //////// write state machine
   parameter DDR2_WR_IDLE =    4'b1 << 0;
   parameter DDR2_WR_DATA =    4'b1 << 1;
   parameter DDR2_WR_CMD  =    4'b1 << 2;
   parameter DDR2_WR_WAIT =    4'b1 << 3;

   parameter DDR2_WR_nSTATES = 4;

   reg [(DDR2_WR_nSTATES-1):0] DDR2_WR_cstate = {{(DDR2_WR_nSTATES-1){1'b0}}, 1'b1};
   reg [(DDR2_WR_nSTATES-1):0] DDR2_WR_nstate;

   always @ (posedge c3_clk0) begin
      if (c3_rst0)
	DDR2_WR_cstate <= DDR2_WR_IDLE; 
      else
	DDR2_WR_cstate <= DDR2_WR_nstate;
   end

   always @ (*) begin
      case (DDR2_WR_cstate) //synthesis parallel_case full_case
	DDR2_WR_IDLE: begin
	   // trigger rising edge of command & rdrw == 0
	   if( (!ddr2_docmd_d && ddr2_docmd) && !ddr2_rdwr ) begin
	      DDR2_WR_nstate = DDR2_WR_DATA;
	   end else begin
	      DDR2_WR_nstate = DDR2_WR_IDLE;
	   end
	end
	DDR2_WR_DATA: begin
	   DDR2_WR_nstate = DDR2_WR_CMD;
	end
	DDR2_WR_CMD: begin
	   DDR2_WR_nstate = DDR2_WR_WAIT;
	end
	DDR2_WR_WAIT: begin
	   if( !ddr2_wr_empty ) begin
	      DDR2_WR_nstate = DDR2_WR_WAIT;
	   end else begin
	      DDR2_WR_nstate = DDR2_WR_IDLE;
	   end
	end
      endcase // case (DDR2_WR_cstate)
   end

   always @ (posedge c3_clk0) begin
      if( c3_rst0 ) begin
	 ddr2_wr_cmd_en <= 1'b0;
	 ddr2_wren <= 1'b0;
      end else begin
	 case (DDR2_WR_cstate) //synthesis parallel_case full_case
	   DDR2_WR_IDLE: begin
	      ddr2_wr_cmd_en <= 1'b0;
	      ddr2_wren <= 1'b0;
	   end
	   DDR2_WR_DATA: begin
	      ddr2_wr_cmd_en <= 1'b0;
	      ddr2_wren <= 1'b1;
	   end
	   DDR2_WR_CMD: begin
	      ddr2_wr_cmd_en <= 1'b1;
	      ddr2_wren <= 1'b0;
	   end
	   DDR2_WR_WAIT: begin
	      ddr2_wr_cmd_en <= 1'b0;
	      ddr2_wren <= 1'b0;
	   end
	 endcase // case (DDR2_WR_cstate)
      end // else: !if( ddr2_reset )
   end // always @ (posedge c3_clk0)

   ///////////
   //////// read state machine
   parameter DDR2_RD_IDLE =    4'b1 << 0;
   parameter DDR2_RD_DATA =    4'b1 << 1;
   parameter DDR2_RD_CMD  =    4'b1 << 2;
   parameter DDR2_RD_WAIT =    4'b1 << 3;

   parameter DDR2_RD_nSTATES = 4;

   reg [(DDR2_RD_nSTATES-1):0] DDR2_RD_cstate = {{(DDR2_RD_nSTATES-1){1'b0}}, 1'b1};
   reg [(DDR2_RD_nSTATES-1):0] DDR2_RD_nstate;

   always @ (posedge c3_clk0) begin
      if (c3_rst0)
	DDR2_RD_cstate <= DDR2_RD_IDLE; 
      else
	DDR2_RD_cstate <= DDR2_RD_nstate;
   end

   always @ (*) begin
      case (DDR2_RD_cstate) //synthesis parallel_case full_case
	DDR2_RD_IDLE: begin
	   // trigger rising edge of command & rdrw == 1
	   if( (!ddr2_docmd_d && ddr2_docmd) && ddr2_rdwr ) begin
	      DDR2_RD_nstate = DDR2_RD_CMD;
	   end else begin
	      DDR2_RD_nstate = DDR2_RD_IDLE;
	   end
	end
	DDR2_RD_CMD: begin
	   DDR2_RD_nstate = DDR2_RD_WAIT;
	end
	DDR2_RD_WAIT: begin
	   if( ddr2_rd_avail_n == 1'b0 ) begin
	      DDR2_RD_nstate = DDR2_RD_DATA;
	   end else begin
	      DDR2_RD_nstate = DDR2_RD_WAIT;
	   end
	end
	DDR2_RD_DATA: begin
	   if( ddr2_rd_avail_n == 1'b0 ) begin
	      DDR2_RD_nstate = DDR2_RD_DATA;
	   end else begin
	      DDR2_RD_nstate = DDR2_RD_IDLE;
	   end
	end
      endcase // case (DDR2_RD_cstate)
   end

   always @ (posedge c3_clk0) begin
      if( c3_rst0 ) begin
	 ddr2_rd_cmd_en <= 1'b0;
	 ddr2_rden <= 1'b0;
      end else begin
	 case (DDR2_RD_cstate) //synthesis parallel_case full_case
	   DDR2_RD_IDLE: begin
	      ddr2_rd_cmd_en <= 1'b0;
	      ddr2_rden <= 1'b0;
	   end
	   DDR2_RD_CMD: begin
	      ddr2_rd_cmd_en <= 1'b1;
	      ddr2_rden <= 1'b0;
	   end
	   DDR2_RD_WAIT: begin
	      ddr2_rd_cmd_en <= 1'b0;
	      ddr2_rden <= 1'b0;
	   end
	   DDR2_RD_DATA: begin
	      ddr2_rd_cmd_en <= 1'b0;
	      ddr2_rden <= 1'b1;
	   end
	 endcase // case (DDR2_RD_cstate)
      end // else: !if( ddr2_reset )
   end // always @ (posedge c3_clk0)
`endif
   
  //////////////////////////////////////
  // cheezy low speed clock divider source
  //////////////////////////////////////
   reg [22:0] counter;

   always @(posedge clk26buf) begin
      counter <= counter + 1;

`ifdef HDMI
      HDCP_AKSV <= Aksv14_write; // retime it into this domain to not screw up timing closure
`endif
   end
   
   
   ////////////////////////////////
   // serial number
   ////////////////////////////////
   reg 	clk1M_unbuf;
   always @(posedge clk26buf) begin
      clk1M_unbuf <= counter[6];
   end
   
   BUFG clk1M_buf(.I(clk1M_unbuf), .O(clk1M));

   wire dna_reset;
   sync_reset  dna_reset_sync(
			  .clk(clk1M),
			  .glbl_reset(glbl_reset),
			  .reset(dna_reset) );
   
   reg 	dna_pulse;
   reg 	dna_shift;
   wire dna_bit;
   reg [55:0] dna_data;
   
   DNA_PORT device_dna( .CLK(clk1M), .DIN(1'b0), .DOUT(dna_bit), .READ(dna_pulse), .SHIFT(dna_shift) );
   
   parameter DNA_INIT =    4'b1 << 0;
   parameter DNA_PULSE =   4'b1 << 1;
   parameter DNA_SHIFT =   4'b1 << 2;
   parameter DNA_DONE =    4'b1 << 3;

   parameter DNA_nSTATES = 4;

   reg [(DNA_nSTATES-1):0] DNA_cstate = {{(DNA_nSTATES-1){1'b0}}, 1'b1};
   reg [(DNA_nSTATES-1):0] DNA_nstate;
   reg [5:0] 		   dna_shift_count;

   always @ (posedge clk1M) begin
      if (dna_reset)
	DNA_cstate <= DNA_INIT; 
      else
	DNA_cstate <= DNA_nstate;
   end

   always @ (*) begin
      case (DNA_cstate) //synthesis parallel_case full_case
	DNA_INIT: begin
	   DNA_nstate = DNA_PULSE;
	end
	DNA_PULSE: begin
	   DNA_nstate = DNA_SHIFT;
	end
	DNA_SHIFT: begin
	   // depending on if MSB or LSB first, want to use 56 or 55
	   // especially if serial #'s are linear-incrementing
	   DNA_nstate = (dna_shift_count[5:0] == 6'd55) ? DNA_DONE : DNA_SHIFT;
	end
	DNA_DONE: begin
	   DNA_nstate = DNA_DONE;
	end
      endcase // case (DNA_cstate)
   end
   
   always @ (posedge clk1M) begin
      if( dna_reset ) begin
	   dna_shift_count <= 6'h0;
	   dna_data <= 56'h0;
	   dna_pulse <= 1'b0;
	   dna_shift <= 1'b0;
      end else begin
	 case (DNA_cstate) //synthesis parallel_case full_case
	   DNA_INIT: begin
	      dna_shift_count <= 6'h0;
	      dna_data <= 56'h0;
	      dna_pulse <= 1'b0;
	      dna_shift <= 1'b0;
	   end
	   DNA_PULSE: begin
	      dna_shift_count <= 6'h0;
	      dna_data <= 56'h0;
	      dna_pulse <= 1'b1;
	      dna_shift <= 1'b0;
	   end
	   DNA_SHIFT: begin
	      dna_shift_count <= dna_shift_count + 6'b1;
	      dna_data[55:0] <= {dna_data[54:0],dna_bit};
	      dna_pulse <= 1'b0;
	      dna_shift <= 1'b1;
	   end
	   DNA_DONE: begin
	      dna_shift_count <= dna_shift_count;
	      dna_data[55:0] <= dna_data[55:0];
	      dna_pulse <= 1'b0;
	      dna_shift <= 1'b0;
	   end
	 endcase // case (DNA_cstate)
      end // else: !if( dna_reset )
   end // always @ (posedge clk1M or posedge ~rstbtn_n)

   ////////////////////////////////
   // heartbeat
   ////////////////////////////////
   pwm heartbeat(.clk812k(clk1M), .pwmout(blue_led),
		 .bright(12'b0000_1111_1000), .dim(12'b0000_0001_0000) );

`ifdef HDMI   
   assign FPGA_LED = !blue_led | HPD_N;
`else
   assign FPGA_LED = !blue_led;
`endif


   
   ////////////////////////////////////////////////////////////////////////////////////
   //// I2C internal control wiring ////
   /////////////////
   /// register 0: control snoop state (SNOOP_CTL r/w)
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         | MODEBNK |         |         | HPD_FRC | SQUASH  |  RD_STB |  RD_HDCP
   //
   //  bit 0 - RD_HDCP. When 1, select readback of the HDCP set; when 0, select EDID set
   //  bit 1 - RD_STB. When high, update the contents of SNOOP_DAT with data at SNOOP_ADR
   //  bit 2 - enable EDID squashing
   //  bit 3 - when high, force HPD to show that nothing is plugged in; low, act as normal
   //  bit 6 - sets the bank to write with register 0x13 (yah yah this is a hack)
   //
   /////////////////
   /// register 1: snoop readback address (SNOOP_ADR r/w)
   //  bits 7:0 are the address to read back  from the snoop unit
   //
   /////////////////
   /// register 2: snoop readback data (SNOOP_DAT ro)
   //  bits 7:0 are the data corresponding to the last loaded snoop address as
   //     selected by RD_HDCP bits in SNOOP_CTL when RD_STB was last toggled
   //
   //  REVISION -- now dynamically relays the value specified by snoop_adr without having
   //     to toggle RD_STB. RD_STB has no effect currently.
   /////////////////
   //  register 3: Compositing control (COMP_CTL r/w)
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  KM_SEM | RST_GNLK| SMRTLCK | RST_PLL |  SELF   | COMP_ON |  KM_RDY |  HDCP_ON
   //
   //  bit 0 - enable HDCP encryption of the composited stream. Enables HDCP encryption
   //          only if the HDCP cipher is used. If the input stream has no HDCP on it
   //          then this bit has no meaning.
   //  bit 1 - indicates that Km is ready and loaded (currently ignored)
   //  bit 2 - enable compositing of incoming data from LCD port to HDMI stream
   //  bit 3 - when set, ignore incoming data and generate sync based on LCD port signals
   //  bit 4 - when set, resets PLLs only on the paths designated by bit3 ("self")
   //  bit 5 - when set, enable "smart locking", i.e., genlock turns off once we're locked
   //  bit 6 - reset the genlock control machine
   //  bit 7 - Km semaphore -- used to indicate to the kernel whether an existing process
   //          is controlling the Km derivation process. This is to resolve the issue where
   //          the HPD will generate two events to cover Km generation in the case that
   //          the final protocol requires a "restart" to get the correct Km value to stick
   //
   /////////////////

   /////////////////
   //  registers 4-7: unused
   //
   
   /////////////////
   //  register 8-B: read-only debug registers, meaning reserved
   //
   
   /////////////////
   //  register C: extended control set (EXT1_CTL r/w)
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  ALPH2  | ALPH1   | ALPH0   |  ALPHEN |         |         | CHROMA  |
   //
   //  bit 7-5: alpha value, 3 bits.
   //  bit 4: alpha blending enable
   //  bit 1: when set, turn on chroma keying; otherwise, always blend in
   //
   /////////////////
   //  NOTE: these are now hard-wired to 240, 0, 240. These registers will likely be deprecated soon.
   //  register D: chroma R value, 8 bits
   //  register E: chroma G value, 8 bits
   //  register F: chroma B value, 8 bits
   //
   /////////////////
   //  register 0x10 is read-only:
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  HDCPDET| VSYNCPOL| HSYNCPOL| LOWVOLT |         |  CEC    | LOCKED  |  BEND
   //  bit 0: chumby_bend pin state (tied off to insure configuratios as an input)
   //  bit 1: indicates that the genlock machine has locked LCD to HDMI streams
   //  bit 2: CEC pin state (tied off to insure configuratios as an input)
   //  bit 4: when high indicates that a low voltage condition was detected; only active during condition
   //         there is also an interrupt to the CPU that fires
   //  bit 5: indicates the polarity of the HSYNC detected on the HDMI stream, 1 = active high
   //  bit 6: indicates the polarity of the VSYNC detected on the HDMI stream, 1 = active high
   //  bit 7: when high, indicates that an HDCP stream is being encrypted. Not active during
   //         horiz and vert sync periods.
   //
   /////////////////
   //  register 0x11-12:
   //    lock tolerance, in pixels, in little-endian byte order
   //    This defines the tolerance of the "lock" threshold. This value matters mostly when
   //    "smart locking" is turned on, i.e., when we want to disable genlock once we're within
   //    our locking tolerance window.
   //
   /////////////////
   //  register 0x13 is address of modeline RAM to write
   //  7 is a write strobe (write when high)
   //  6:0 is the actual modeline address
   //
   /////////////////
   //  register 0x14 is the data to write into the modeline RAM
   //  7:0 is the data
   //
   /////////////////
   //  registers 0x15-0x17:
   //    lock target count, in pixels, in little-endian byte order.
   //    Lock target count is the amount of time that the LCD interface should lead the
   //    HDMI inteface for producing data. The amount of time should be large enough to
   //    absorb timing variations in the interrupt latency handling of the PXA168, but
   //    in all cases smaller than 8 lines of video (that's the size of the line buffers).
   //
   //    The total time should be expressed in units of pixels, not lines.
   //
   /////////////////
   //  register 0x18 is read-only:
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  RX_VLD |  B_RDY  |  G_RDY  |  R_RDY  | ALGNERR |  SYNLCK | TXLCK   |  RXLCK
   //  bit 0: rx PLL is locked, indicates that a clock is present and cable is plugged in
   //  bit 1: tx PLL is locked. This should generally always be the case.
   //  bit 2: synthesizer DCM is locked. This should generally always be the case.
   //  bit 3: rx alignment error
   //  bit 4: red channel has received a valid pixel
   //  bit 5: green chanel has received a valid pixel
   //  bit 6: blue channel has received a valid pixel
   //  bit 7: all RX chanels are phase aligned and producing valid data
   //   
   /////////////////
   //  registers 0x19-0x1f: Km
   //  56-bit value of Km, entered in little-endian order.
   //
   /////////////////
   //  All registers after this point are read-only, and byte-order is little-endian
   /////////////////
   //  register 0x20-21: horizontal active in pixels
   //  register 0x22-23: vertical active in lines
   //  register 0x24-25: horizontal total width in pixels
   //  register 0x26-28: vertical total height in pixels (not lines)
   //  register 0x29: horizontal front porch in pixels
   //  register 0x2a: horizontal back porch in pixels
   //  register 0x2b-2d: vertical front porch in pixels (not lines)
   //  register 0x2e-30: vertical back porch in pixels (not lines)
   //  register 0x31: horizontal sync width in pixels
   //  register 0x32-0x34: vertical sync width in pixels (not lines)
   //  register 0x35-0x37: reference clock count cycles
   //
   //  register 0x38-0x3e: device ID (7 bytes)
   //
   //  register 0x3f: version number. Note that value 0xFF means to refer to extended version number
   

   ///////////////// EXTENDED REGISTER SET
   //  registers with addresses 0x40-0x7F are write-only
   //  registers with addresses 0x80-0xFF are read-only
   /////////////////
   //
   //  register 0x40: digital output values for digital bits 7:0
   //  register 0x41: output enable for digital bits 7:0
   //  register 0x42: pullup enables for digital bits 7:0
   //  register 0x43: analog pullup enables for analog bits 7:0
   //
   /////////////////
   //  register 0x45: digital shift chain control
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |         |         |  RESET  |  SAMPLE |  UPDTE
   //  bit 0: on rising edge, update the digital pins with the loaded register values
   //  bit 1: on rising edge, sample all the digital inputs simultaneously
   //  bit 2: main system reset
   //
   /////////////////
   //  register 0x46: ADC control
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |    GO   |  CHAN3  |  CHAN2  |  CHAN1 |  CHAN0
   //  bits 2-0: channel of ADC converter to convert
   //  bit 3: 1 selects user ADC, 0 selects motor current fbk ADC
   //
   /////////////////
   //  register 0x47: motor control
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |         |         |         |         |  ALLSTP
   //  bit 0: when set, all motors are immediately stopped
   //
   /////////////////
   //  register 0x48: motor direction
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  M3D1   |  M3D0   |  M2D1   |  M2D0   |  M1D1   |  M1D0   |  M0D1   |  M0D0
   //  for all direction 2-bit codes, 10 = forward, 01 = reverse, 11 = short brake, 00 = stop
   //  note: short brake shorts both motor terminals to ground
   //        stop just shorts one terminal to ground
   //  bits 1-0: motor 0 direction
   //  bits 3-2: motor 1 direction
   //  bits 5-4: motor 2 direction
   //  bits 7-6: motor 3 direction
   //  
   /////////////////
   //  register 49-4a: motor PWM duty cycle (MDUTY)
   //  12-bit value specifying the duty cycle of the motor PWM
   //  A 12-bit value allows a minimu duty cycle resolution of 1/4095 = 0.02%
   //
   /////////////////
   //  register 4c-4b: motor PWM divider (MDIV)
   //  16-bit vaule specifying the divider for the motor PWM
   //  The base clock for the motor PWM is 208MHz / 4096 (from 12 bits res). 
   //  The final period for the PWM is thus:
   //  period = 50.78kHz / ( MDIV + 2)
   //  period * MDIV + 2 * period = 50.78kHz
   //  (50.78kHz - 2 * period) / period = MDIV
   //
   /////////////////
   //  register 4d-4f: servo PWM period (SPERIOD)
   //  24-bit value which specifies the length of the PWM period for the servo
   //  Servo PWM is custom-built for specifying sparse, high-resolution narrow pulse widths.
   //  The period for the servo is defined to be:
   //  13 MHz / (SPERIOD + 1)
   //  As SPERIOD is a 24-bit number, the longest period is thus 0.75 Hz.
   //
   /////////////////
   //  register 50-52: servo 0 pulse width (S0PULSE)
   //  24-bit value which specifies the width of the servo pulse. The quanta for the value is
   //  1/13 MHz = 56.8ns
   //  Please note that the pulse width is not a percentage duty cycle, but an absolute time specifier
   //
   /////////////////
   //  register 53-55: servo 1 pulse width (S1PULSE)
   //  see servo 0 description
   //
   /////////////////
   //  register 58-56: servo 2 pulse width (S2PULSE)
   //  see servo 0 description
   //
   /////////////////
   //  register 5b-59: servo 3 pulse width (S3PULSE)
   //  see servo 0 description
   //
   /////////////////
   //  register 0x60-63: 32-bit test data to write to DDR2 (little endian)
   //  register 0x64-67: 32-bit test address for all DDR2 operations (read and write)
   //
   /////////////////
   //  register 0x68 is write-only:
   //  control commands for DDR2 interfaces
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |         |         |         |  DO_CMD |  RD_WR
   //  bit 0: select read or write (1 is read, 0 is write)
   //  bit 1: start the requested command now
   //
   /////////////////
   //
   //  begin read-only setion
   //
   /////////////////
   //  register 0x80 is the digital I/O status register
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |         |         |         |  DGOOD  |  DBUSY
   //  bit 0: 1 indicates that the digital shift chain is busy
   //  bit 1: 1 indicates that the digital input value is good 
   //         (i.e., updated from most recent sample request)
   //
   /////////////////
   //  register 0x81-82 is the ADC value register (lower 10 bits only)
   //
   /////////////////
   //  register 0x83 is the ADC status register
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |         |         |         |         | AVALID
   //  bit 0: 1 means that the ADC in value is valid. Cleared when ADC "GO" is triggered.
   //         insensitive to changes on channel specifier.
   //
   //
   /////////////////
   //  register 0x84 is the digital interface input levels
   //
   /////////////////
   //  register 0x90-93 is read-only: 32-bit read data from DDR2
   //
   /////////////////
   //
   //  register 0x94 is the status for DDR2 control interface
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         |         |         |         |         | CALDONE | WREMPTY | RDAVAIL
   //  bit 0: indicates that read data is available
   //  bit 1: indicates that the write FIFO is empty
   //  bit 2: indicates that the DDR2 interface's calibration is done
   //
   /////////////////
   //  register 0x95 is for DDR2 state machine debug
   //  bits 7-4: wr machine
   //  bits 3-0: rd machine
   //
   /////////////////
   //  register 0xfc-0xff are the extended version code
   //  0xfc-0xfd is the implementation revision (16 bits)
   //  0xfe-0xff is the machine code (16 bits):
   //     0x0000: reserved
   //     0x0001: Kovan
   //

   wire [7:0] reg_addr;
   wire       wr_stb;
   wire [7:0] reg_data_in;
   wire [7:0] reg_a2;
   wire [7:0] snoop_ctl;
   wire [7:0] snoop_rbk_adr;
   wire [7:0] snoop_rbk_dat;

`ifdef HDMI
   // this snippet allows us to clean up an ugly special case in the original i2c engine implementation
   // what we wanted was a holding register. What was originally implemented was adding a mux to 
   // the *entire* register set! So we move the holding register one level up and simplified things.
   reg [7:0]  reg_data_holding;
   reg 	      wr_stb_d;
   always @(posedge clk26buf) begin
      wr_stb_d <= wr_stb;
      if (wr_stb & !wr_stb_d) begin // only act on the rising pulse of wr_stb
	 reg_data_holding <= reg_data_in;
      end else begin
	 reg_data_holding <= reg_data_holding;
      end
   end
`endif
   
   wire       SDA_pd;
   wire       SDA_int;
   IOBUF #(.DRIVE(8), .SLEW("SLOW")) IOBUF_sda (.IO(XI2CSDA), .I(1'b0), .T(!SDA_pd), .O(SDA_int));

   i2c_slave host_i2c(
		      .SCL(XI2CSCL),
		      .SDA(SDA_int),
		      .SDA_pd(SDA_pd),

		      .clk(clk26buf),
		      .glbl_reset(glbl_reset),

		      .i2c_device_addr(8'h3C),
`ifdef HDMI
		      .wr_stb(wr_stb),
		      .reg_0(snoop_ctl),
		      .reg_1(snoop_rbk_adr),
		      .reg_2(reg_data_holding),
		      .reg_3(comp_ctl),
`endif
		      
		      .reg_8(8'h34),  // reg8-b are placeholders
		      .reg_9(8'h0D), 
		      .reg_a(8'hBA),
		      .reg_b(8'hBE),

`ifdef HDMI
		      .reg_c(ext1_ctl),
		      
		      // hard-wired to 240, 0, 240
		      // reg_d-f were chroma, now hard-wired to 240, 0, 240
		      
		      .reg_10({hdcp_requested,hdmi_vsync_pol,hdmi_hsync_pol,LOWVOLT_NOTIFY,1'b0,CEC,
			       genlock_locked, CHUMBY_BEND}),


		      .reg_11(lock_tolerance[7:0]),
		      .reg_12(lock_tolerance[15:8]),
		      
		      .reg_13({modeline_write,modeline_adr[6:0]}),
		      .reg_14(modeline_dat),

		      .reg_15(target_lead_pixels[7:0]),
		      .reg_16(target_lead_pixels[15:8]),
		      .reg_17(target_lead_pixels[23:16]),

		      .reg_18({rx_all_valid,
			       rx0_blue_rdy, rx0_green_rdy, rx0_red_rdy,
			       rx0_psalgnerr,
			       m720p_locked, tx0_plllckd, rx0_plllckd}),
			       
		      .reg_19(Km[7:0]),
		      .reg_1a(Km[15:8]),
		      .reg_1b(Km[23:16]),
		      .reg_1c(Km[31:24]),
		      .reg_1d(Km[39:32]),
		      .reg_1e(Km[47:40]),
		      .reg_1f(Km[55:48]),
`endif //  `ifdef HDMI

`ifdef HDMI
		      //// read-only registers after this point
		      .reg_20(t_hactive[7:0]),
		      .reg_21({4'b0,t_hactive[11:8]}),
		      .reg_22(t_vactive[7:0]),
		      .reg_23({4'b0,t_vactive[11:8]}),
		      .reg_24(t_htotal[7:0]),
		      .reg_25({4'b0,t_htotal[11:8]}),
		      .reg_26(t_vtotal[7:0]),
		      .reg_27(t_vtotal[15:8]),
		      .reg_28(t_vtotal[23:16]),
		      .reg_29(t_h_fp[7:0]),
		      .reg_2a(t_h_bp[7:0]),
		      .reg_2b(t_v_fp[7:0]),
		      .reg_2c(t_v_fp[15:8]),
		      .reg_2d(t_v_fp[23:16]),
		      .reg_2e(t_v_bp[7:0]),
		      .reg_2f(t_v_bp[15:8]),
		      .reg_30(t_v_bp[23:16]),
		      .reg_31(t_hsync_width[7:0]),
		      .reg_32(t_vsync_width[7:0]),
		      .reg_33(t_vsync_width[15:8]),
		      .reg_34(t_vsync_width[23:16]),
		      .reg_35(t_refclkcnt[7:0]),
		      .reg_36(t_refclkcnt[15:8]),
		      .reg_37(t_refclkcnt[23:16]),
`endif //  `ifdef HDMI

		      .reg_38(dna_data[7:0]),
		      .reg_39(dna_data[15:8]),
		      .reg_3a(dna_data[23:16]),
		      .reg_3b(dna_data[31:24]),
		      .reg_3c(dna_data[39:32]),
		      .reg_3d(dna_data[47:40]),
		      .reg_3e(dna_data[55:48]),

		      .reg_3f(8'hFF),    // version number

		      // extended register space:
		      // reg 40 - reg 80 are write registers (64 locations)
		      // reg 80 - reg C0 are read-only registers (64 locations)
		      // write-only interfaces
		      .reg_40(dig_out_val),
		      .reg_41(dig_oe),
		      .reg_42(dig_pu),
		      .reg_43(ana_pu),
		      // reg_44 unused
		      .reg_45({glbl_reset_edge,dig_sample,dig_update}),
		      .reg_46({adc_go,adc_chan[3:0]}),
		      .reg_47({mot_allstop}),
		      .reg_48(mot_drive_code),
		      .reg_49(mot_pwm_duty[7:0]),
		      .reg_4a(mot_pwm_duty[15:8]),
		      .reg_4b(mot_pwm_div[7:0]),
		      .reg_4c(mot_pwm_div[15:8]),
		      
		      .reg_4d(servo_pwm_period[7:0]),
		      .reg_4e(servo_pwm_period[15:8]),
		      .reg_4f(servo_pwm_period[23:16]),
		      
		      .reg_50(servo0_pwm_pulse[7:0]),
		      .reg_51(servo0_pwm_pulse[15:8]),
		      .reg_52(servo0_pwm_pulse[23:16]),

		      .reg_53(servo1_pwm_pulse[7:0]),
		      .reg_54(servo1_pwm_pulse[15:8]),
		      .reg_55(servo1_pwm_pulse[23:16]),

		      .reg_56(servo2_pwm_pulse[7:0]),
		      .reg_57(servo2_pwm_pulse[15:8]),
		      .reg_58(servo2_pwm_pulse[23:16]),

		      .reg_59(servo3_pwm_pulse[7:0]),
		      .reg_5a(servo3_pwm_pulse[15:8]),
		      .reg_5b(servo3_pwm_pulse[23:16]),
		      
		      .reg_60(ddr2_write_data[7:0]),
		      .reg_61(ddr2_write_data[15:8]),
		      .reg_62(ddr2_write_data[23:16]),
		      .reg_63(ddr2_write_data[31:24]),
		      .reg_64(ddr2_test_addr[7:0]),
		      .reg_65(ddr2_test_addr[15:8]),
		      .reg_66(ddr2_test_addr[23:16]),
		      .reg_67(ddr2_test_addr[29:24]),
		      .reg_68(ddr2_regcmd[7:0]),

		      // reg_0x78 - reg_0x7f reserved for loopback testing
		      // read-only interfaces
		      .reg_80({6'b0,dig_val_good, dig_busy}),
		      .reg_81(adc_in[7:0]),
		      .reg_82({6'b000000,adc_in[9:8]}),
		      .reg_83({7'b0,adc_valid}),
		      .reg_84(dig_in_val[7:0]),

		      .reg_90(ddr2_read_data[7:0]),
		      .reg_91(ddr2_read_data[15:8]),
		      .reg_92(ddr2_read_data[23:16]),
		      .reg_93(ddr2_read_data[31:24]),
		      .reg_94(ddr2_regstat[7:0]),
		      .reg_95(ddr2_sm_dbg[7:0]),

		      /// extened version -- 32 bits to report versions
		      /// kovan starts at FF.00.01.00.01
		      .reg_fc(8'h3),  // this is the LSB of the extended version field
		      .reg_fd(8'h0),
		      .reg_fe(8'h1),
		      .reg_ff(8'h0)   // this is the MSB of the extended version field
		      );
     
   /////// version FF.0001.0004 (log created 3/6/2012)
   //
   
   /////// version FF.0001.0003 (log created 3/1/2012)
   // - fix ADC bit width
   // - fix digital input data bit position
   // - fix serial number bug
   // - increase motor PWM base clock rate to 208 MHz to allow for 10kHz 12-bit PWM control
   // - decrease DNA clock rate to 1MHz; eliminate 2MHz clock line
   // - clean up documentation around motor and servo dividers
   // - fix ADC channel select bit position
   // - all motor control functions validated

   /////// version FF.0001.0002 (log created 3/1/2012)
   // - reverse direction of LRCLK to accommodate ES8328 codec restrictions
   
   /////// version FF.0001.0001 changes (log created 2/18/2012)
   // - branch to kovan
   // - FF means to check auxiliary vesion field
   // - initial checkin of code

   ////////////////////////////// legacy changes
   /////// version 4 changes
   // - added input registers to LCD path to clean up timing
   // - added a pipeline stage to the core video processing pipe
   // - adjusted the position of chroma decision versus chroma to remove right pink stripe
   // - fixed chroma to 240, 0, 240 to reduce computational complexity
   // - fixed timing files to get better coverage of the FPGA
   // - inverted clock to device DNA state machine to fix hold time race condition
   // - self-timed mode is now native, no need to switch FPGA configs
   // - added PLL and alignment/valid feedback registers to detect when source is present
   // - touch-up clock tree to improve clarity of timing definition & rule propagation
   // - full switch-over to PlanAhead tool for compilation

   /////// version 5 changes (log created 8/11/2011)
   // - changed blue LED from flashing to breathing

   /////// version 6 changes (log created 8/12/2011)
   // - added off state to LED which is automatically triggered when output not plugged in
   
   /////// version 7 changes (log created 8/12/2011)
   // - added SOURCE_NOTIFY reporting trigger
   // - removed HPD debounce circuit, so HPD reports even when no source is present

   /////// version 8 changes (log cerated 8/21/2011)
   // - changed timing detector to always report the timing of the Rx stream, even in self-timed mode

   /////// version 9 changes (log created 8/22/2011)
   // - removed setbox functionality. Box is always "full screen". 
   // Registers 4-B are now deprecated (they are NOPs if written in this implementation, may
   // change to be active later).

   /////// version A changes (log created 8/22/2011)
   // - HDCP cipher now resets properly when Ksv is double-initialized
   // - EDID snooper for HDCP now limits read bounds to 5 to compensate
   //   for tivo series 3 weirdness.

   /////// version B changes (log created 8/27/2011)
   // - timing closure only

   /////// version C changes (log created 8/27/2011)
   // - fix chroma issue in overlay mode, was checking too many bits, causing jagged edges on photos
   
   /////// version D changes (log created 9/5/2011)
   // - add workaround for Apple TV 2 EESS bug. ATV2 asserts EESS by far too early. Added a trap to catch
   //   EESS when it is asserted too early relative to vsync

   /////// version E changes (log created 9/29/2011)
   // - fix RGB color depth issue; turns out that extending the LSB's isn't the right way to do it.
   //   now, we truncate the unused bits to zero

   /////////////// dummy tie-downs
`ifdef HDMI

`else
   // dummy tie-downs to make UCF constraints happy when there is no HDMI interface
   wire [3:0] dummy_tmds;
   
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy0 (.I(RX0_TMDS_P[0]), .IB(RX0_TMDS_N[0]), .O(dummy_tmds[0]));
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy1 (.I(RX0_TMDS_P[1]), .IB(RX0_TMDS_N[1]), .O(dummy_tmds[1]));
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy2 (.I(RX0_TMDS_P[2]), .IB(RX0_TMDS_N[2]), .O(dummy_tmds[2]));
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy3 (.I(RX0_TMDS_P[3]), .IB(RX0_TMDS_N[3]), .O(dummy_tmds[3]));

   // just for testing for sean
   OBUFDS TMDS0 (.I(1'b0), .O(TX0_TMDS_P[0]), .OB(TX0_TMDS_N[0])) ;
   OBUFDS TMDS1 (.I(1'b0), .O(TX0_TMDS_P[1]), .OB(TX0_TMDS_N[1])) ;
   OBUFDS TMDS2 (.I(1'b0), .O(TX0_TMDS_P[2]), .OB(TX0_TMDS_N[2])) ;
   OBUFDS TMDS3 (.I(1'b0), .O(TX0_TMDS_P[3]), .OB(TX0_TMDS_N[3])) ;
//   assign TX0_TMDS_P[0] = I2S_CLK1;
//   assign TX0_TMDS_P[1] = I2S_DI1;
//   assign TX0_TMDS_P[2] = I2S_LRCLK1;
//   assign TX0_TMDS_P[3] = 1'b0;

   assign DDC_SDA_PU = 1'b0;
   assign DDC_SDA_PD = 1'b0;
   
   assign VSYNC_STB = 1'b0;
   assign HPD_OVERRIDE = 1'b0;
`endif // !`ifdef HDMI

`ifdef COMPLETE
   
`else
   // some stand-ins while we complete the code
   
   assign CAM_PCLKI = 1'b0;
   assign CAM_HSYNC = 1'b0;
   assign CAM_VSYNC = 1'b0;
   assign FPGA_MISO = 1'b0;
   assign CAM_D[7:0] = 8'b0;
   assign CHG_SHDN = 1'b1;
`endif
   
endmodule // kovan
