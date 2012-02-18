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
	      input wire        I2SCDCLK0, // master reference clock to audio PLL
	      output wire       I2SCDCLK1,
	      output wire       I2SCLK0,   // return sample clock to CPU
	      input wire        I2SCLK1,
	      input wire        I2SDI0,    // audio data to playback
	      output wire       I2SDI1,
	      output wire       I2SDO0,    // audio data from record
	      input wire        I2SDO1,
	      input wire        I2SLRCLK0, // left/right clock to codec
	      output wire       I2SLRCLK1,

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
	      input wire        FPGA_SCLK,  // this needs to be fixed to 26 MHz
	      input wire        FPGA_SYNC,

	      // I2C interfaces
	      input wire        PWR_SCL,  // we listen on this one
	      inout wire        PWR_SDA,

	      input wire        XI2CSCL,  // our primary interface
	      inout wire        Xi2CSDA,

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
	      output                               mcb3_dram_ck_n
	      );

   ///////// clock buffers
   wire            clk26;
   wire 	   clk26ibuf;
   wire 	   clk26buf;
   wire 	   clk26ibuf;
   
   
   assign clk26 = FPGA_SCLK;
   IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));
   BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));

   
   ////////// reset
   wire 	   glbl_reset; // to be used sparingly


   ////////// loop-throughs
   // lcd runs at a target of 6.41 MHz (156 ns cycle time)
   // i.e., 408 x 262 x 60 Hz (408 is total H width, 320 active, etc.)
   wire            qvga_clkgen_locked;
   
   clk_wiz_v3_2_qvga qvga_clkgen( .CLK_IN1(clk26buf),
				  .CLK_OUT1(clk_qvga),
				  .RESET(glbl_reset),
				  .LOCKED(qvga_clkgen_locked) );
   
   reg 		   lcd_pipe_b[5:0];
   reg 		   lcd_pipe_r[5:0];
   reg 		   lcd_pipe_g[5:0];
   reg 		   lcd_pipe_den;
   reg 		   lcd_hsync;
   reg 		   lcd_vsync;
   reg 		   lcd_reset_n;
   wire 	   lcd_reset;

   sync_reset  qvga_reset(
			  .clk(clk_qvga),
			  .glbl_reset(glbl_reset),
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
			  .Q(LCDO_DOTCLK), .CE(1), .R(0), .S(0) );

   ODDR2 qvga_clk_to_cpu (.D0(1'b1), .D1(1'b0), 
			 .C0(clk_qvga), .C1(!clk_qvga), 
			 .Q(LCD_CLK_T), .CE(1), .R(0), .S(0) );


   // audio pass-through -- unbuffurred, unregistered for now
   assign I2SCDCLK1 = I2SCDCLK0;
   assign I2SCLK0 = I2SCLK1;
   assign I2SDI1 = I2SDI0;
   assign I2SDO0 = I2SDO1;
   assign I2SLRCLK1 = I2SLRCLK0;
   
   // - add I2C unit --> wire up remaining pins to dummy registers
   
   // - motor control unit
   
   /////// DDR2 core 128 MB x 16 of memory, 312 MHz (624 Mt/s = 1.2 GiB/s)
   // generate a 312 MHz clock from the 26 MHz local buffer
   wire c3_sys_clk;
   wire c3_clk_locked;
   clk_ddr2_26m_312m ddr2_clk(
			      .CLK_IN1(clk26buf),
			      .CLK_OUT1(c3_sys_clk),
			      .RESET(glbl_reset),
			      .LOCKED(c3_clk_locked)
			      );

   // instantiate the core
   wire c3_clk0; // clock to internal fabric
   wire c3_rst0;
   wire [31:0] ddr2_read_data;
   reg [29:0]  ddr2_test_addr;

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
			    .c3_sys_rst_i(!c3_clk_locked),
			    .c3_calib_done(c3_calib_done),
			    .c3_clk0(c3_clk0), 
			    .c3_rst0(c3_rst0), 

			    // internal interfaces. only two brought out here for testing
			    // port 2 - read-only
			    .c3_p2_cmd_clk(c3_clk0),
			    .c3_p2_cmd_en(1'b1),
			    .c3_p2_cmd_instr(3'b011),
			    .c3_p2_cmd_bl(6'b1),
			    .c3_p2_cmd_byte_addr(ddr2_test_addr),
			    .c3_p2_rd_clk(c3_clk0),
			    .c3_p2_rd_en(ddr2_test_addr[2]),
			    .c3_p2_rd_data(ddr2_read_data),

			    // port 3 - write-only
			    .c3_p3_cmd_clk(c3_clk0),
			    .c3_p3_cmd_en(1'b1),
			    .c3_p3_cmd_instr(3'b010),
			    .c3_p3_cmd_bl(6'b1),
			    .c3_p3_cmd_byte_addr(~ddr2_test_addr),
			    .c3_p3_wr_clk(c3_clk0),
			    .c3_p3_wr_en(!ddr2_test_addr[2]),
			    .c3_p3_wr_mask(4'hf),
			    .c3_p3_wr_data(ddr2_test_addr + 32'haa55)
			    );
   
   always @(posedge c3_clk0) begin
      ddr2_test_addr <= ddr2_test_addr + 1;
   end
   
   assign ddr2_dummy = ^ ddr2_read_data;


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
   reg clk2M_unbuf;
   (* clock_signal = "yes" *)
   (* PERIOD = "period 0.8125 MHz" *)
   wire clk2M;
   wire clk1M;
   reg 	clk1M_unbuf;
   always @(posedge clk26buf) begin
      clk2M_unbuf <= counter[4]; // 0.8MHz clock: device DNA only runs at 2 MHz
      clk1M_unbuf <= counter[6];
   end
   
   BUFG clk2M_buf(.I(clk2M_unbuf), .O(clk2M));
   BUFG clk1M_buf(.I(clk1M_unbuf), .O(clk1M));

   wire dna_reset;
   sync_reset  dna_reset(
			  .clk(clk_2M),
			  .glbl_reset(glbl_reset),
			  .reset(dna_reset) );
   
   reg 	dna_pulse;
   reg 	dna_shift;
   wire dna_bit;
   DNA_PORT device_dna( .CLK(clk2M), .DIN(1'b0), .DOUT(dna_bit), .READ(dna_pulse), .SHIFT(dna_shift) );
   
   parameter DNA_INIT =    4'b1 << 0;
   parameter DNA_PULSE =   4'b1 << 1;
   parameter DNA_SHIFT =   4'b1 << 2;
   parameter DNA_DONE =    4'b1 << 3;

   parameter DNA_nSTATES = 4;

   reg [(DNA_nSTATES-1):0] DNA_cstate = {{(DNA_nSTATES-1){1'b0}}, 1'b1};
   reg [(DNA_nSTATES-1):0] DNA_nstate;
   reg [5:0] 		   dna_shift_count;

   always @ (posedge clk2M) begin
      if (dna_reset)
	DNA_cstate <= DNA_INIT; 
      else
	DNA_cstate <=#1 DNA_nstate;
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
   
   always @ (posedge clk2M) begin
      if( dna_reset ) begin
	   dna_shift_count <= 6'h0;
	   dna_data <= 56'h0;
	   dna_pulse <= 1'b0;
	   dna_shift <= 1'b0;
      end else begin
	 case (DNA_cstate) //synthesis paralell_case full_case
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
   end // always @ (posedge clk2M or posedge ~rstbtn_n)

   ////////////////////////////////
   // heartbeat
   ////////////////////////////////
   pwm heartbeat(.clk812k(clk1M), .pwmout(blue_led),
		 .bright(12'b0000_1111_1000), .dim(12'b0000_0001_0000) );

`ifdef HDMI   
   assign FPGA_LED = blue_led | HPD_N;
`else
   assign FPGA_LED = blue_led;
`endif

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
   //  register 0x3f: version number
   

   //////////// DEPRACATED REGISTERS ////////////// (but not confident they are dead yet)
   //  register D: line empty/full levels
   //  This sets the level at which we declare the line buffers to be "empty" or "full"
   //  This is on a the granularity of a full video line, not at the pixel level. The
   //  pixel-level full/empty is hard-coded by the FIFO primitive.
   //  bit 7 is ignored
   //  bits [6:4] are the full level target: nominally 2
   //  bit 3 is ignored
   //  bits [2:0] are the empty level target: nominally 2
   //
   /////////////////
   //  register E: write and read initial levels
   //  This sets the initial state of the write/read pointers, reset on every vsync.
   //  Note that the actual bit vector that keeps track of active lines can be much longer
   //  than 4 bits, but we only get to diddle with the bottom four bits in this implementation.
   //  bits [7:4] are for the write: nominally 1
   //  bits [3:0] are for the read: nominally 2
   //
   /////////////////
   //  register F: reserved
   //

   wire SDA_pd;
   wire [7:0] reg_addr;
   wire       wr_stb;
   wire [7:0] reg_data_in;
   wire [7:0] reg_a2;
   wire       SDA_int;
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
   
   IOBUF #(.DRIVE(8), .SLEW("SLOW")) IOBUF_sda (.IO(SDA), .I(1'b0), .T(!SDA_pd), .O(SDA_int));

   i2c_slave host_i2c(
		      .SCL(SCL),
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
		      // reg 40 - reg 60 are write registers (32 locations, growable to 64)
		      // reg 80 - reg C0 are read-only registers (64 locations)
		      .reg_40( ), /// control something (written by host)
		      .reg_80( ), /// readback something (read by host)

		      /// extened version -- 32 bits to report versions
		      /// kovan starts at FF.000000
		      .reg_fc(8'h0),  // this is the LSB of the extended version field
		      .reg_fd(8'h0),
		      .reg_fe(8'h0),
		      .reg_ff(8'h0)   // this is the MSB of the extended version field
		      );
     
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

   /////// version FF.00000000 changes (log create 2/18/2012)
   // - branch to kovan
   // - FF means to check auxiliary vesion field
   
   
endmodule // kovan
