module reset_synch(

	input wire RST_n, clk,
	output wire rst_n
);

// synchronize the reset signal with two flops
reg ff1, ff2;

assign rst_n = ff2;

// reset will asynch reset the flops, upon RST_n going high,
// then 1'b1 will propagate synchronously to the output proving a synchronized reset
always @(negedge clk, negedge RST_n)
	if(!RST_n)
		ff2 <= 1'b0;
	else
		ff2 <= ff1;

always @(negedge clk, negedge RST_n)
	if(!RST_n)
		ff1 <= 1'b0;
	else
		ff1 <= 1'b1;
		


endmodule
