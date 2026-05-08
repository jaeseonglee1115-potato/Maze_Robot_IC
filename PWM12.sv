module PWM12(

    input wire clk, rst_n,
    input wire [11:0]duty,
    output reg PWM1, PWM2

);

logic set_1, rset_1, set_2, rset_2;
reg [11:0]cnt;
logic [11:0]sum_cnt_NONOVERLAP;
localparam NONOVERLAP = 12'h02C;


///////////////////////////////////////////////////////////////////////////////////////////////////
// Set/reset logic for PWM1 and 2 ensures that there is no cross conduction in the H-Bridge
///////////////////////////////////////////////////////////////////////////////////////////////////
always_comb begin 

    // PWM1 logic
    set_1 = cnt >= NONOVERLAP & cnt < duty;
        
    // Turn off when its duty-time has run out
    rset_1 = cnt >= duty;
    

    // PWM2 logic
    sum_cnt_NONOVERLAP = duty + NONOVERLAP;

    // Tun on if count is greater-equal the duty + nonoverlap time AND overflow did not occur.
    // this allows changing of duty cycle immedately.
    set_2 = cnt >= sum_cnt_NONOVERLAP & !(duty[11] & ~sum_cnt_NONOVERLAP[11]);
    rset_2 = &cnt;
end

///////////////////////////////////////////////////////////////////////////////////////////////////
// Synchronous up counter keeps track of PWM cycle
///////////////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk, negedge rst_n) begin

    if(!rst_n) begin
        cnt <= 12'h000;
    end else begin 
        cnt <= cnt + 1;
    end

end

///////////////////////////////////////////////////////////////////////////////////////////////////
// Synchonizes and stores the current output of PWM1 - prevents glitching
///////////////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk, negedge rst_n ) begin 
    
    if(!rst_n) begin // Asynch reset
        PWM1 <= 1'b0;
    end else if (set_1 || rset_1) begin
        PWM1 <= set_1 & ~rset_1;
    end
end

///////////////////////////////////////////////////////////////////////////////////////////////////
// Synchonizes and stores the current output of PWM2 - prevents glitching
///////////////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk, negedge rst_n ) begin 
    
    if(!rst_n) begin // Asynch reset
        PWM2 <= 1'b0;
    end else if (set_2 || rset_2) begin
        PWM2 <= set_2 & ~rset_2;
    end
end

endmodule