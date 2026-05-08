module UART_wrapper (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        RX,
    output logic        TX,
    output logic        cmd_rdy,
    output logic [15:0] cmd,
    input  logic        clr_cmd_rdy,
    input  logic        trmt,
    input  logic [7:0]  resp,
    output logic        tx_done
);


//logic clr_rdy; // for clearing uart buffer
logic [7:0] rx_data, upper_byte, lower_byte; // for receiving data from uart
logic sel_upper; // for shifting data into shift register and indicating new msb received
typedef enum logic [1:0] {RECEIVING_H, RECEIVING_L} state_r;
logic set_rdy; 
state_r state, nxt_state;
logic clr_rx_rdy;
logic rx_rdy;

assign cmd = {upper_byte, lower_byte};
 // uart sv instatiation
UART iUART (
    .clk(clk),
    .rst_n(rst_n),
    .RX(RX),
    .TX(TX),
    .rx_rdy(rx_rdy),
    .clr_rx_rdy(clr_rx_rdy),
    .rx_data(rx_data),
    .trmt(trmt),
    .tx_data(resp),
    .tx_done(tx_done)
);


always_comb begin

  nxt_state =  state;  // default state is current state
  clr_rx_rdy = 0; // default clear ready is 0
  set_rdy = 0; // default set ready is 0
  sel_upper = 1'b0;

    case (state)

    RECEIVING_H: begin

        if (rx_rdy) begin
            sel_upper = 1'b1;
            clr_rx_rdy = 1;
            nxt_state = RECEIVING_L;
        end
    end

    RECEIVING_L: begin
        // second byte already in rx_data
        if (rx_rdy) begin
                clr_rx_rdy = 1;
                set_rdy    = 1;
                nxt_state = RECEIVING_H;
      end
    end
endcase
end



// from state to state transitions
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= RECEIVING_H;  // RECEIVING_H
    end else begin
        state <= nxt_state;  // get next state
    end
end

always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        lower_byte <= 8'h00;
        upper_byte <= 8'h00;
    end
    else if (sel_upper && rx_rdy) begin
        upper_byte <= rx_data;
     end
    else if (!sel_upper && rx_rdy) begin
        lower_byte <= rx_data;
    end

end

// latch apparently stops cmmand from glitching 
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n)
        cmd_rdy <= 0;
    else if (set_rdy)
        cmd_rdy <= 1;
    else if (clr_cmd_rdy)
        cmd_rdy <= 0;
end
endmodule
