module SPI_main(

input logic clk, rst_n, MISO, wrt,
input logic [15:0]wt_data,
output logic done, SS_n, SCLK, MOSI,
output logic [15:0] rd_data

);

typedef enum logic [2:0] {IDLE, F_PORCH, COMM} state_sp;

state_sp state, nxt_state;

logic MISO_smpl, ld_SCLK, shift_imm, init, shift, set_done, smpl, done15;
logic [3:0] bit_cnt;
logic [4:0] clk_cnt;

assign MOSI = rd_data[15];

// Make a shift register that parallel stores data to transmit
// and serially shifts in received data from MISO
always_ff @(posedge clk) begin

	if (init) 
		rd_data <= wt_data;
	else if (shift)
		rd_data <= {rd_data[14:0], MISO_smpl};
end

// we must store the value of MISO on the rising edge of SCLK so it can
// be shifted in on the falling edge of SCLK
always_ff @(posedge clk) begin
	if(smpl) MISO_smpl <= MISO;
end

// assert done15 to indicate 16 shifts have occured
assign done15 = &bit_cnt;

// counter stores the number of shifts performed
always_ff @(posedge clk) begin

	if(init)
		bit_cnt <= 4'h0;
	else if (shift)
		bit_cnt <= bit_cnt + 4'h1;
end

// SCLK is the MSB of the clock counter, which means it will
// operate at 1/32 system freq. smpl will be high a clk cycle before
// SLK goes high. smpl_imm will go high a clk cycle before SCLK goes low
assign SCLK = clk_cnt[4];
assign smpl = &clk_cnt[3:0] & ~clk_cnt[4];
assign shift_imm = &clk_cnt;

// create counter to keep track of SCLK and CLK
always_ff @(posedge clk) begin
	if(ld_SCLK)
		clk_cnt <= 5'h17;
	else
		clk_cnt <= clk_cnt + 5'h01;
end

// Flop prevents gliching of done
always_ff @(posedge clk, negedge rst_n) begin

	if(!rst_n) begin
		SS_n <= 1'b1;
	end else if(set_done || init)
		SS_n <= set_done & ~init;
end

always_ff @(posedge clk, negedge rst_n) begin

	if(!rst_n) begin
		done <= 1'b0;
	end else if(set_done || init)
		done <= set_done & ~init;
end

always_comb begin

	shift = 0;
	init = 0;
	ld_SCLK = 1;
	set_done = 0;
	nxt_state = state;

	case (state)
		
		// when starting a new write cycle enable SCLK counter and initialize
		IDLE : if (wrt) begin
			init = 1;
			nxt_state = F_PORCH;
		end

		F_PORCH : begin
			ld_SCLK = 0;
			if (shift_imm) begin
				nxt_state = COMM;
			end
		end

		// The SCLK counter will be active throughout the SPI transmission. 
		// Upon end of transmission, SCLK counter will be disabled in order
		// to create back porch.
		COMM: begin
			ld_SCLK = 0;
			if(done15 && shift_imm) begin
				shift = 1;
				set_done = 1;
				ld_SCLK = 1;
				nxt_state = IDLE;
				// if we aren't at the first falling edge, then shift new value
			end else if (!done15 && shift_imm) begin
				shift = 1;
			end
		end
		default : 
			nxt_state = IDLE;
	endcase

end

// FSM state flop
always_ff @(posedge clk, negedge rst_n) begin

	if(!rst_n)
		state <= IDLE;
	else
		state <= nxt_state;
end

endmodule
