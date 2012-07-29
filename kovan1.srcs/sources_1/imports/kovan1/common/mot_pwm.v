module mot_pwm(
	       input wire clk,
	       input wire [PWM_PRECISION_WIDTH-1:0] duty_cycle,
	       output wire pwm_output
	       );
   
   parameter PWM_PRECISION_WIDTH = 12;

   reg [PWM_PRECISION_WIDTH-1:0] duty_cycle_reg, temp_reg, old_dc;
   reg [PWM_PRECISION_WIDTH-1:0] pwm_count;
   
   reg 				 new_duty_cycle;
   reg 				 pwm_state;
   reg 				 got_new_dc;
   
   // synchronize changes to local register if d/c changes
   always @ (posedge clk) begin
      if (~|pwm_count) begin
         duty_cycle_reg <= duty_cycle; // only update when pwm_count is at 0 state
      end else begin
	 duty_cycle_reg <= duty_cycle_reg;
      end
   end

   // now PWM
   always @(posedge clk) begin
      pwm_count <= pwm_count + 1;
      if( &duty_cycle_reg ) begin
	 pwm_state <= 1'b1; // if duty cycle is 100%, don't glitch
      end else if( duty_cycle_reg > pwm_count ) begin
	 pwm_state <= 1'b1;
      end else begin
	 pwm_state <= 1'b0;
      end
   end // always @ (posedge clk or posedge reset)
   
   assign pwm_output = pwm_state;
   
endmodule // mot_pwm
