//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Andrew "bunnie" Huang
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation and/or 
//    other materials provided with the distribution.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
//    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
//    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
//    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
//    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
//    POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////////

`timescale 1 ns / 1 ps

module robot_iface(
	     input wire  clk,
	     input wire  clk_3p2MHz,
	     input wire  clk_208MHz,
	     input wire  glbl_reset,

	     // digital i/o block
	     input wire [7:0] dig_out_val,
	     input wire [7:0] dig_oe,
	     input wire [7:0] dig_pu,
	     input wire [7:0] ana_pu,
	     output reg [7:0] dig_in_val,
	     output wire      dig_val_good, // output value is valid when high
	     output wire      dig_busy,    // chain is busy when high
	     input wire       dig_sample,  // samples input on rising edge
	     input wire       dig_update,  // updates chain on rising edge

	     // ADC interface
	     output reg [9:0] adc_in,
	     input wire [3:0] adc_chan,    // channels 0-7 are for user, 8-15 are for motor current fbk
	     output reg       adc_valid,
	     input wire       adc_go,  

	     // motor driver interface
	     input wire [15:0] mot_pwm_div,
	     input wire [15:0] mot_pwm_duty,
	     input wire [7:0]  mot_drive_code, // 2 bits/chan, 00 = stop, 01 = forward, 10 = rev, 11 = stop
	     input wire        mot_allstop,

	     // servo interface
	     input wire [23:0] servo_pwm_period, // total period for the servo update
	     input wire [23:0] servo0_pwm_pulse, // pulse width in absolute time
	     input wire [23:0] servo1_pwm_pulse,
	     input wire [23:0] servo2_pwm_pulse,
	     input wire [23:0] servo3_pwm_pulse,

	     /////// physical interfaces to outside the chip
	     // motors
	     output reg [3:0] MBOT,
	     output reg [3:0] MTOP,
	     output wire       MOT_EN,
	     output wire [3:0] M_SERVO,

	     // analog interface
	     output wire [1:0] DIG_ADC_CS,
	     output wire       DIG_ADC_IN,
	     input wire        DIG_ADC_OUT,
	     output wire       DIG_ADC_SCLK,

	     // digital interface
	     output wire       DIG_IN,
	     input wire        DIG_OUT,
	     output wire       DIG_RCLK,
	     output wire       DIG_SAMPLE,
	     output wire       DIG_SCLK,
	     output wire       DIG_SRLOAD,
	     output wire       DIG_CLR_N
	     );

   ///////////////// digital i/o interface and pull-up control
   reg 			 go;
   reg 			 go_d;
   reg 			 go_edge;

   reg 			 dinsamp;
   
   wire 		 motor_reset;
   sync_reset  motor_reset_sync(
			  .clk(clk),
			  .glbl_reset(glbl_reset),
			  .reset(motor_reset) );

   assign DIG_CLR_N = !motor_reset;
   assign DIG_SAMPLE = dinsamp;
   
   always @(posedge clk) begin
      go <= dig_update;
      go_d <= go;
      go_edge <= go & !go_d;
	 
      dinsamp <= dig_sample;
   end // always @ (posedge clk)

   parameter SR_INIT           =   5'b1 << 0;
   parameter SR_START_DIG      =   5'b1 << 1;
   parameter SR_SHIFT_DIG      =   5'b1 << 2;
   parameter SR_SHIFT_DIG_TERM =   5'b1 << 3;
   parameter SR_DIG_DONE       =   5'b1 << 4;

   parameter SR_nSTATES = 5;

   reg [(SR_nSTATES-1):0] SR_cstate = {{(SR_nSTATES-1){1'b0}}, 1'b1};
   reg [(SR_nSTATES-1):0] SR_nstate;
   
   reg [5:0] 		  shift_count;
   reg 			  dig_srload;
   reg [39:0] 		  shift_in;
   reg [31:0] 		  shift_out;
   reg 			  update_dig;
   reg 			  busy;
   reg 			  dgood;

      
   always @ (negedge clk) begin
      if (motor_reset)
	SR_cstate <= SR_INIT; 
      else
	SR_cstate <= SR_nstate;
   end

   always @ (*) begin
      case (SR_cstate) //synthesis parallel_case full_case
	SR_INIT: begin
	   if( go_edge ) begin 
	      SR_nstate = SR_START_DIG;
	   end else begin
	      SR_nstate = SR_INIT;
	   end
	end // case: SR_INIT

	SR_START_DIG: begin
	   SR_nstate = SR_SHIFT_DIG;
	end

	SR_SHIFT_DIG: begin
	   SR_nstate = (shift_count[5:0] == 6'h1e) ? SR_SHIFT_DIG_TERM : SR_SHIFT_DIG;
	end

	SR_SHIFT_DIG_TERM: begin
	   SR_nstate = SR_DIG_DONE;
	end
	
	SR_DIG_DONE: begin
	   SR_nstate = SR_INIT;
	end

      endcase // case (SR_cstate)
   end

   assign dig_busy = busy;
   assign dig_val_good = dgood;

   // 0x40 ff ff ff  (locating input data on shift chain, in value = 0x81)
   //  30       23        15                0
   // 0100 0000 1111 1111 1111 1111 1111 1111
   
   //// note, we assume that DIG_SAMPLE is driven by user before this chain is triggered
   //// the split enables very precise user control over when sampling happens, versus readout
   wire [7:0] dig_oe_n;
   assign dig_oe_n[7:0] = ~dig_oe[7:0];
   
   always @ (posedge clk) begin
      case (SR_cstate) //synthesis parallel_case full_case
	SR_INIT: begin
	   shift_count <= 6'b0;
	   shift_in <= 40'b0;
	   shift_out <= 32'b1111_1111_1111_1111_1111_1111_1111_1111;
	   
	   dig_srload <= 1'b1;
	   update_dig <= 1'b1;
	   dig_in_val <= dig_in_val;

	   if( dinsamp ) begin
	      dgood <= 1'b0;
	   end else begin
	      dgood <= dgood;
	   end
	   
	   busy <= busy;
	end
	SR_START_DIG: begin
	   shift_count <= 6'b0;
	   shift_out <= {ana_pu[7:0],dig_pu[7:0],dig_oe_n[7:0],dig_out_val[7:0]};  
	   shift_in <= shift_in;

	   dig_srload <= 1'b0; // just need to create a rising edge on the next transition
	   update_dig <= 1'b1;
	   dig_in_val <= dig_in_val;
	   
	   dgood <= 1'b0;
	   busy <= 1'b1;
	end
	SR_SHIFT_DIG: begin
	   shift_count <= shift_count + 6'b1;
	   shift_in[39:0] <= {shift_in[38:0],DIG_OUT};
	   shift_out <= {shift_out[30:0],1'b1};
	   
	   dig_srload <= 1'b1;
	   update_dig <= 1'b1;
	   dig_in_val <= dig_in_val;

	   dgood <= dgood;
	   busy <= 1'b1;
	end
	SR_SHIFT_DIG_TERM: begin
	   shift_count <= shift_count + 6'b1;
	   shift_in[39:0] <= {shift_in[38:0],DIG_OUT};
	   shift_out <= {shift_out[30:0],1'b1};
	   
	   dig_srload <= 1'b1;
	   update_dig <= 1'b0;
	   dig_in_val <= dig_in_val;
	   
	   dgood <= dgood;
	   busy <= 1'b1;
	end
	SR_DIG_DONE: begin
	   shift_count <= shift_count;
	   shift_in <= shift_in;
	   shift_out <= shift_out;
	   
	   dig_srload <= 1'b1;
	   update_dig <= 1'b1;
	   dig_in_val <= shift_in[30:23];
	   
	   dgood <= 1'b1;
	   busy <= 1'b0;
	end
      endcase // case (SR_cstate)
   end // always @ (posedge clk)
   
   ODDR2 dig_clk_mirror (.D0(1'b0), .D1(1'b1),  // not inversion of clk
			 .C0(clk), .C1(!clk), 
			 .Q(DIG_SCLK), .CE(1'b1), .R(1'b0), .S(1'b0) );
   
   assign DIG_IN = shift_out[31];
   assign DIG_RCLK = update_dig;
   
   // TODO: check if srload is glitch-free, as it is asynchronous
   assign DIG_SRLOAD = dig_srload;

   
   /////////////////////////////////
   //////////////////////// PWM block
   /////////////////////////////////
   
   ///////// motor PWM prescaler
   reg pwm_clk;
   reg pwm_clk_a;
   reg [15:0] pwm_scaler;
   wire      pwmclk;
   
   always @(posedge clk_208MHz) begin
      if( (pwm_scaler > mot_pwm_div) || (&pwm_scaler) ) begin
	 pwm_scaler <= 0;
	 pwm_clk_a <= 1;
      end else begin
	 pwm_clk_a <= 0;
	 pwm_scaler <= pwm_scaler + 1;
      end
   end // always @ (posedge clk or posedge reset)

   always @(posedge clk_208MHz) begin
      pwm_clk <= pwm_clk_a; // just to clean everything up, no glitches on clock
   end

   BUFG pwmclkbuf(.I(pwm_clk), .O(pwmclk));

   ///////// PWM for motors -- one PWM for all four channels
   mot_pwm mot1_chan (.clk(pwmclk),
		     .duty_cycle(mot_pwm_duty[11:0]), // note only 12 bits used, set in lower module
		     .pwm_output(MOT_EN));

   ///////// motor direction controls
   always @(posedge clk) begin
      MBOT[0] <= ((mot_drive_code[1:0] == 2'b10) ? 1'b1 : 1'b0) & !mot_allstop;
      MTOP[0] <= ((mot_drive_code[1:0] == 2'b01) ? 1'b1 : 1'b0) & !mot_allstop;
      
      MBOT[1] <= ((mot_drive_code[3:2] == 2'b10) ? 1'b1 : 1'b0) & !mot_allstop;
      MTOP[1] <= ((mot_drive_code[3:2] == 2'b01) ? 1'b1 : 1'b0) & !mot_allstop;

      MBOT[2] <= ((mot_drive_code[5:4] == 2'b10) ? 1'b1 : 1'b0) & !mot_allstop;
      MTOP[2] <= ((mot_drive_code[5:4] == 2'b01) ? 1'b1 : 1'b0) & !mot_allstop;

      MBOT[3] <= ((mot_drive_code[7:6] == 2'b10) ? 1'b1 : 1'b0) & !mot_allstop;
      MTOP[3] <= ((mot_drive_code[7:6] == 2'b01) ? 1'b1 : 1'b0) & !mot_allstop;
   end
   
   // servo channels
   wire [3:0] m_servo_out;
   servo_pwm servo0_chan (.clk(clk),
			  .period(servo_pwm_period),
			  .pulse(servo0_pwm_pulse),
			  .pwm_output(m_servo_out[0]));

   servo_pwm servo1_chan (.clk(clk),
			  .period(servo_pwm_period),
			  .pulse(servo1_pwm_pulse),
			  .pwm_output(m_servo_out[1]));

   servo_pwm servo2_chan (.clk(clk),
			  .period(servo_pwm_period),
			  .pulse(servo2_pwm_pulse),
			  .pwm_output(m_servo_out[2]));

   servo_pwm servo3_chan (.clk(clk),
			  .period(servo_pwm_period),
			  .pulse(servo3_pwm_pulse),
			  .pwm_output(m_servo_out[3]));

   assign M_SERVO[0] = !m_servo_out[0]; // invert to compensate inverting level converters
   assign M_SERVO[1] = !m_servo_out[1];
   assign M_SERVO[2] = !m_servo_out[2];
   assign M_SERVO[3] = !m_servo_out[3];

   
   /////////////////////////////////
   //////////////////////// ADC block
   /////////////////////////////////
   parameter ADC_INIT           =   5'b1 << 0;
   parameter ADC_START          =   5'b1 << 1;
   parameter ADC_SHIFT          =   5'b1 << 2;
   parameter ADC_SHIFT_TERM     =   5'b1 << 3;
   parameter ADC_DONE           =   5'b1 << 4;

   parameter ADC_nSTATES = 5;

   reg [(ADC_nSTATES-1):0] ADC_cstate = {{(ADC_nSTATES-1){1'b0}}, 1'b1};
   reg [(ADC_nSTATES-1):0] ADC_nstate;

   wire 		   motor_reset_3p2;
   sync_reset  motor_reset_sync_3p2(
			  .clk(clk_3p2MHz),
			  .glbl_reset(glbl_reset),
			  .reset(motor_reset_3p2) );


   reg 			   adc_go_d;
   reg 			   adc_go_edge;
   reg [4:0] 		   adc_shift_count;
   reg [15:0] 		   adc_shift_out;
   reg [15:0] 		   adc_shift_in;
   reg [1:0] 		   adc_cs;
   
   always @(posedge clk_3p2MHz) begin
      adc_go_d <= adc_go;
      adc_go_edge <= adc_go & !adc_go_d;
   end // always @ (posedge clk)
   
   always @ (posedge clk_3p2MHz) begin
      if (motor_reset_3p2)
	ADC_cstate <= ADC_INIT; 
      else
	ADC_cstate <= ADC_nstate;
   end

   always @ (*) begin
      case (ADC_cstate) //synthesis parallel_case full_case
	ADC_INIT: begin
	   if( adc_go_edge ) begin 
	      ADC_nstate = ADC_START;
	   end else begin
	      ADC_nstate = ADC_INIT;
	   end
	end // case: ADC_INIT

	ADC_START: begin
	   ADC_nstate = ADC_SHIFT;
	end

	ADC_SHIFT: begin
	   ADC_nstate = (adc_shift_count[4:0] == 5'he) ? ADC_SHIFT_TERM : ADC_SHIFT;
	end

	ADC_SHIFT_TERM: begin
	   ADC_nstate = ADC_DONE;
	end
	
	ADC_DONE: begin
	   ADC_nstate = ADC_INIT;
	end

      endcase // case (ADC_cstate)
   end
   

   always @ (posedge clk_3p2MHz) begin
      case (ADC_cstate) //synthesis parallel_case full_case
	ADC_INIT: begin
	   adc_shift_count <= 5'b0;
	   adc_shift_out <= 32'b1111_1111_1111_1111;
	   adc_shift_in <= 16'b0;
	   adc_cs <= 2'b11;
	   
	   adc_valid <= adc_valid;
	   adc_in <= adc_in;
	end
	ADC_START: begin
	   adc_shift_count <= 5'b0;
//	   adc_shift_out <= {11'b0,adc_chan[0],adc_chan[1],adc_chan[2],2'b0};
	   adc_shift_out <= {2'b0,adc_chan[2],adc_chan[1],adc_chan[0],11'b0};
	   adc_shift_in <= adc_shift_in;
	   adc_cs <= adc_chan[3] ? 2'b01 : 2'b10;
	   
	   adc_valid <= 0;
	   adc_in <= adc_in;
	end
	ADC_SHIFT: begin
	   adc_shift_count <= adc_shift_count + 6'b1;
	   adc_shift_out <= {adc_shift_out[14:0],1'b1};
	   adc_shift_in[15:0] <= {adc_shift_in[14:0], DIG_ADC_OUT};
	   adc_cs <= adc_cs;
	   
	   adc_valid <= 0;
	   adc_in <= adc_in;
	end
	ADC_SHIFT_TERM: begin
	   adc_shift_count <= adc_shift_count + 6'b1;
	   adc_shift_out <= {adc_shift_out[14:0],1'b1};
	   adc_shift_in[15:0] <= {adc_shift_in[14:0], DIG_ADC_OUT};
	   adc_cs <= adc_cs;
	   
	   adc_valid <= 0;
	   adc_in <= adc_in;
	end
	ADC_DONE: begin
	   adc_shift_count <= adc_shift_count;
	   adc_shift_out <= adc_shift_out;
	   adc_shift_in <= adc_shift_in;
	   adc_cs <= 2'b11;
	   
	   adc_valid <= 1;
	   adc_in <= adc_shift_in[11:2];
	end
      endcase // case (ADC_cstate)
   end // always @ (posedge clk)

   ODDR2 adc_clk_mirror (.D0(1'b0), .D1(1'b1),   // note inversion of clk_3p2MHz
			 .C0(clk_3p2MHz), .C1(!clk_3p2MHz), 
			 .Q(DIG_ADC_SCLK), .CE(1'b1), .R(1'b0), .S(1'b0) );
   
   assign DIG_ADC_IN = adc_shift_out[15];
   assign DIG_ADC_CS[1:0] = adc_cs[1:0];
   
endmodule // robot_iface


