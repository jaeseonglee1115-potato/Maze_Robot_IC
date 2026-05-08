module navigate(clk,rst_n,strt_hdng,strt_mv,stp_lft,stp_rght,mv_cmplt,hdng_rdy,moving,
			en_fusion,at_hdng,lft_opn,rght_opn,frwrd_opn,frwrd_spd);
			
	parameter FAST_SIM = 1;				// speeds up incrementing of frwrd register for faster simulation
				
	input clk,rst_n;					// 50MHz clock and asynch active low reset
	input strt_hdng;					// indicates should start a new heading
	input strt_mv;						// indicates should start a new forward move
	input stp_lft;						// indicates should stop at first left opening
	input stp_rght;						// indicates should stop at first right opening
	input hdng_rdy;						// new heading reading ready....used to pace frwrd_spd increments
	output logic mv_cmplt;				// asserted when heading or forward move complete
	output logic moving;				// enables integration in PID and in inertial_integrator
	output en_fusion;					// Only enable fusion (IR reading affect on nav) when moving forward at decent speed.
	input at_hdng;						// from PID, indicates heading close enough to consider heading complete.
	input lft_opn,rght_opn,frwrd_opn;	// from IR sensors, indicates available direction.  Might stop at rise of lft/rght
	output reg [10:0] frwrd_spd;		// unsigned forward speed setting to PID

	localparam MAX_FRWRD = 11'h2A0;		// max forward speed
	localparam MIN_FRWRD = 11'h0D0;		// minimum duty at which wheels will turn

	typedef enum reg [2:0] {IDLE, FORWARD, STOP_FAST, STOP, TURN, TURN_CMPLT} state_c;
	state_c state, nxt_state;

	reg prev_lft_opn, prev_rght_opn;
	logic lft_opn_rise, rght_opn_rise, init_frwrd, inc_frwrd, dec_frwrd, dec_frwrd_fast;
	wire [5:0] frwrd_inc;

	////////////////////////////////
	// Now form forward register //
	//////////////////////////////
	always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		frwrd_spd <= 11'h000;
	else if (init_frwrd)
		frwrd_spd <= MIN_FRWRD;									// min speed to get motors moving
	else if (hdng_rdy && inc_frwrd && (frwrd_spd<MAX_FRWRD))	// max out at 400 of 7FF for control head room
		frwrd_spd <= frwrd_spd + {5'h00,frwrd_inc};
	else if (hdng_rdy && (frwrd_spd>11'h000) && (dec_frwrd | dec_frwrd_fast))
		frwrd_spd <= ((dec_frwrd_fast) && (frwrd_spd>{2'h0,frwrd_inc,3'b000})) ? frwrd_spd - {2'h0,frwrd_inc,3'b000} : // 8x accel rate
					(dec_frwrd_fast) ? 11'h000 :	  // if non zero but smaller than dec amnt set to zero.
					(frwrd_spd>{4'h0,frwrd_inc,1'b0}) ? frwrd_spd - {4'h0,frwrd_inc,1'b0} : // slow down at 2x accel rate
					11'h000;

	// We only care about new openning, so we must edge detect in order to only activate on new openning.
	always_ff @( posedge clk ) begin 
		prev_lft_opn <= lft_opn;
	end

	always_ff @( posedge clk ) begin 
		prev_rght_opn <= rght_opn;
	end

	// edge has risen if the prev was low and current is high
	assign lft_opn_rise = ~prev_lft_opn & lft_opn;
	assign rght_opn_rise = ~prev_rght_opn & rght_opn;


	// FAST_SIM parameter for simulation will speed up simulation time by using a 
	// larger acceleration and deceleration increment
	generate 
		if (FAST_SIM) begin

			assign frwrd_inc = 6'h18;

		end else begin

			assign frwrd_inc = 6'h02;
		end
	endgenerate


	// en_fusion if the magnitude of frwrd_spd is more than 1/2 MAX_FRWRD
	assign en_fusion = (frwrd_spd > (MAX_FRWRD>>1));



	// FSM for controlling navigate unit
	always_comb begin
		moving =1'b0;
		mv_cmplt = 1'b0;
		init_frwrd = 1'b0;
		dec_frwrd = 1'b0;
		dec_frwrd_fast = 1'b0;
		inc_frwrd = 1'b0;
		nxt_state = state;

		case (state) 

			// Idle robot shoult not be moving
			IDLE : if (strt_mv && frwrd_opn) begin

				nxt_state = FORWARD;
				init_frwrd = 1'b1;
				moving = 1'b1;

			end else if (strt_hdng) begin

				nxt_state = TURN;
				moving = 1'b1;
			end

			// stp_lft and stp_rght are modes of operation. Ex: stp_lft means the robot will look
			// for a a new left openning and then initiate a turn.
			FORWARD : if ((lft_opn_rise && stp_lft || rght_opn_rise && stp_rght) && frwrd_opn) begin

				nxt_state = STOP;
				moving = 1'b1;
			end else if (!frwrd_opn) begin

				nxt_state = STOP_FAST;
				inc_frwrd = 1'b1;
				moving = 1'b1;
			end else begin
				
				inc_frwrd = 1'b1;
				moving = 1'b1;

			end

			// This will state lowers speed by 2x inc 
			STOP : if(frwrd_spd == 0) begin
				
				nxt_state = IDLE;
				mv_cmplt = 1'b1;
				
			end else if (!frwrd_opn) begin

				nxt_state = STOP_FAST;
				moving = 1'b1;

			end else begin
			
				dec_frwrd = 1'b1;
				moving = 1'b1;

			end

			// The path forward is blocked and we must stop at 8x the normal rate
			STOP_FAST : if(frwrd_spd == 0) begin
				
				nxt_state = IDLE;
				mv_cmplt = 1'b1;
				
			end else begin

				dec_frwrd_fast = 1'b1;
				moving = 1'b1;

			end

			// Keep asserting moving when turning until the heading is reached.
			TURN : begin
				moving = 1'b1;
				if(at_hdng)
					nxt_state = TURN_CMPLT;
			end
			// This state isolates at_hdng for synthesis
			// This Turn_cmplt is implemented to delay the mv_cmplt signal by one cycle while pipelining
			TURN_CMPLT : begin
				nxt_state = IDLE;
				mv_cmplt = 1'b1;
			end


			// If in an error state, go back to idle
			default : 
				nxt_state = IDLE;

		endcase
	end

	// Flop stores the current state and assigns next state every clock cycle
	always_ff @(posedge clk, negedge rst_n) begin

		if(!rst_n) 
			state <= IDLE;
		else 
			state <= nxt_state;

	end

endmodule