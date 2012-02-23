// servo PWM module
// basically, you have a major period, which is approx. 20 ms (50Hz)
// and then a pulse, which has a tmin ~ 1ms duration
// and a tmax ~ 2ms duration
// and you want to subdivide the pulse by some number of ticks

// we have a 26 MHz clock available which should theoretically give 
// us 38ns resolution if this is done right.

// let's assume the 26 MHz clock is a given, fixed, and invariant.
//
// first, we need to have a mechanism to tell us when 20 ms has passed.
// when that has passed, we then want to count out a period of time
// which corresponds to tmin + pwmval * delta, 
// where pwmval * delta < (tmax - tmin)

module servo_pwm(
		 input wire clk, // 26 MHz
		 input wire [23:0] period,
		 input wire [23:0] pulse,
		 output wire pwm_output
	       );
   
   reg [23:0] 		     period_cnt;
   reg 			     pwm_state;
   reg 			     pwm_deglitch;
   
   always @(posedge clk) begin
      if( period_cnt[23:0] >= period[23:0] ) begin
	 period_cnt <= 24'h0;
      end else begin
	 period_cnt <= period_cnt + 1;
      end
   end // always @ (posedge clk or posedge reset)

   always @(posedge clk) begin
      if( period_cnt > pulse ) begin
	 pwm_state <= 1;
      end else begin
	 pwm_state <= 0;
      end
      
      pwm_deglitch <= pwm_state;
   end // always @ (posedge clk or posedge reset)

   assign pwm_output = !pwm_deglitch;
   
endmodule // servo_pwm

