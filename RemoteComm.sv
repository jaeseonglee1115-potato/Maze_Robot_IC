module RemoteComm(

input RX, snd_cmd, clk, rst_n,
input [15:0]cmd,
output logic cmd_snt,
output resp_rdy,
output TX,
output [7:0]resp,
output logic [7:0] tx_data
);

logic S, R, set_cmd_snt,sel_high,trmt, tx_done; // for sending command
assign R = snd_cmd;
// set command sent when snd_cmd is high and previous value of snd_cmd is low (rising edge)
assign S = set_cmd_snt; 

//kept clr_rdy as 0 since we are not clearing the uart buffer in this module, 
// just passing the command ready signal to the uart wrapper
UART iUART_trans ( .clk(clk), .rst_n(rst_n),.RX(RX),.TX(TX), .rx_rdy(resp_rdy), .clr_rx_rdy(1'b0),
                    .trmt(trmt),.tx_data(tx_data),.tx_done(tx_done), .rx_data(resp));

typedef enum logic [1:0] {IDLE, TX_HIGH, TX_LOW} state_c;
state_c state, nxt_state;
logic [7:0] tx_low_cmd, tx_high_cmd;

assign tx_data = (sel_high) ? cmd[15:8] :   tx_low_cmd;
// state transition logic
always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) begin
        state <= IDLE;
    end else begin
        state <= nxt_state;
    end

end


// FOR THE CMD_SNT signals
always_ff @( posedge clk , negedge rst_n ) begin
        if(!rst_n) begin
            cmd_snt <= 1'b0;  // asynch rst 
        end else if (S) begin
            cmd_snt <= 1'b1; // set command sent to 1 when S is high
        end else if (R) begin
            cmd_snt <= 1'b0; // if on reset, set command sent to 0
        end
end


always_ff @(posedge clk) begin
    if(snd_cmd) begin
        tx_low_cmd  <= cmd[7:0];
       // tx_high_cmd  <= cmd[15:8];

    end
end

// state transition logic comb
always_comb begin
   trmt = 0; // default transmit is 0
   sel_high = 0; // default select high byte is 0
   set_cmd_snt = 0; // default set command ready is 0
   nxt_state = state;
     case(state)

        IDLE: begin 
            if (snd_cmd) begin
                
                nxt_state = TX_HIGH;  // if send command, move to transmitting high byte state
                trmt = 1; //  set transmit to 1 to transmit high byte, initially
                sel_high = 1; // select high byte to transmit
            end
        end 

        TX_HIGH: begin

            sel_high = 1; // select high byte to transmit

            if (tx_done) begin
                nxt_state = TX_LOW;  // if command sent, move to transmitting low byte state
                sel_high = 0; // select low byte to transmit after high byte is transmitted
                trmt = 1; // if command sent, set transmit to 1 to transmit low byte (this time)
           end
        end 

        TX_LOW: begin

            if (tx_done) begin
                set_cmd_snt = 1; // set command ready when command is sent
                nxt_state = IDLE;  // if command sent, move back to idle state
            end
        end

        default: begin
         nxt_state = IDLE;
        end 
    endcase      
end


endmodule    