///////////////////////////////////////////////////////////////////////////////////////////////////
// This module handles the output of the IR sensors. It handles cases where the right or left side
// of the robot are open, which means that the value read is junk. To compensate, this module
// uses a nominal value NOM_IR as a substitute. For control, we only care about difference in
// Left and right IR sensor reading.
///////////////////////////////////////////////////////////////////////////////////////////////////
module IR_math #(parameter NOM_IR = 12'h900) (
     
    input wire lft_opn, 
    input wire rght_opn, 
    input wire [11:0]lft_IR, 
    input wire [11:0]rght_IR, 
    input wire signed [8:0]IR_Dtrm, 
    input wire en_fusion, 
    input wire signed [11:0]dsrd_hdng, 
    input wire clk,
    output wire [11:0]dsrd_hdng_adj
    
);

wire signed [12:0]IR_diff;
wire signed [12:0]diff_lft_NOM;
wire signed [12:0]diff_NOM_rght;
wire signed [11:0]sel_out;
wire signed [12:0]ext_13_out;
logic signed [12:0]sum_IR_Dtrm, sum_IR_Dtrm_pipe;
wire signed [11:0]sum_dsrd_IR;
wire signed [12:0]mult4_ext13;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Take difference of input signals, we don't need common mode value to control robot.
// both left and right IR signal are UNSIGNED, and should be 0 extended before operation!
///////////////////////////////////////////////////////////////////////////////////////////////////
assign IR_diff = {1'b0,lft_IR} - {1'b0,rght_IR};
assign diff_lft_NOM = lft_IR - NOM_IR;
assign diff_NOM_rght = NOM_IR - rght_IR;

///////////////////////////////////////////////////////////////////////////////////////////////////
// If both IR sensors have valid readings, select the difference between them. If the left is open,
// use nominal IR value and rght IR sensor value. Else, the rght is open and the nominal value 
// and rght IR sensor should be used.
///////////////////////////////////////////////////////////////////////////////////////////////////
assign sel_out = (lft_opn && rght_opn) ? 12'h000 :
        (lft_opn) ? diff_NOM_rght :
        (rght_opn) ? diff_lft_NOM : IR_diff[12:1];

///////////////////////////////////////////////////////////////////////////////////////////////////
// Scale IR_dtrm by 4 and sign extend, then add with the scaled and extended IR sensor difference.
///////////////////////////////////////////////////////////////////////////////////////////////////
assign mult4_ext13 = {{2{IR_Dtrm[8]}},IR_Dtrm[8:0],{2{1'b0}}};
assign sum_IR_Dtrm = {{6{sel_out[11]}},sel_out[11:5]} + mult4_ext13;

always_ff @(posedge clk) begin
    sum_IR_Dtrm_pipe <= sum_IR_Dtrm;
end

assign sum_dsrd_IR = sum_IR_Dtrm_pipe[12:1] + dsrd_hdng;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Fuse IR and Gyro only at high speed
///////////////////////////////////////////////////////////////////////////////////////////////////
assign dsrd_hdng_adj = (en_fusion) ? sum_dsrd_IR : dsrd_hdng;

endmodule