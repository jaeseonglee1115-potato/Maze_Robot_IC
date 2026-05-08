module MtrDrv(
	input signed [11:0]lft_spd, 
	input signed [11:0]rght_spd,
	input [11:0]vbatt,
	input clk, rst_n,
	output lftPWM1, lftPWM2,
	output rghtPWM1, rghtPWM2

);

	// Logic declarations
	logic signed [12:0] scale;
	logic signed [23:0] lft_prod, rght_prod;
	logic signed [23:0] lft_prod_D, rght_prod_D;
	logic signed [11:0] lft_scaled, rght_scaled;
	logic [11:0] lft_duty, rght_duty;

	// Instantiates the duty cycle scaling ROM, and the PWM generators for both motors
	DutyScaleROM dutyLUT(.clk(clk), .batt_level(vbatt[9:4]), .scale(scale));
	PWM12 lftPWM(.PWM1(lftPWM1), .PWM2(lftPWM2), .duty(lft_duty), .clk(clk), .rst_n(rst_n));
	PWM12 rghtPWM(.PWM1(rghtPWM1), .PWM2(rghtPWM2), .duty(rght_duty), .clk(clk), .rst_n(rst_n));

	// Determine the scaled product of the input speed and the duty cycle scaling factor
	assign lft_prod_D = scale * lft_spd;
	assign rght_prod_D = scale * rght_spd;

	// Pipeline Registers before the multiplication 
	always_ff @(posedge clk) begin

		lft_prod <= lft_prod_D;
		rght_prod <= rght_prod_D;

	end

	// Logic to determine the final scaled speed values for the left and right motors. Also handles saturation.
	always_comb begin
	// scaled_factor is always greater than 0
	if (!lft_spd[11] && lft_prod[23])
		lft_scaled = 12'h7FF;
	else
		lft_scaled = lft_prod[23:11];

	if (!rght_spd[11] && rght_prod[23])
		rght_scaled = 12'h7FF;
	else
		rght_scaled = rght_prod[23:11];

	end

	// Decides the duty for left and right that feeds into the PWM
	assign lft_duty = lft_scaled + 12'h800;
	assign rght_duty = 12'h800 - rght_scaled;

endmodule
