module UART_rx (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        RX,
    input  logic        clr_rdy,
    output logic [7:0]  rx_data,
    output logic        rdy
);

// Parameters & Baud Rate Configuration
localparam int BAUD_RATE = 19200;
localparam int BAUD_TIME = 50_000_000 / BAUD_RATE;
localparam int BAUD_MAX  = BAUD_TIME - 1;

// State Machine Definitions
typedef enum logic {
    IDLE,
    RECEIVING
} state_t;

state_t curr_state, nxt_state;

// Internal Registers & Signals Declarations
logic [11:0] baud_cnt;
logic [3:0]  bit_cnt;
logic [8:0]  rx_shift_reg;
logic receiving;
logic RX_sync_0, RX_sync_1;

logic start;
logic S , R;
logic shift;
logic set_rdy;

// Flag for Set/Reset signals
assign S = set_rdy;
assign R =  start;

//////////////////////////////
// Double-Flop Synchronizer //
//////////////////////////////
always_ff @(posedge clk , negedge rst_n) begin
        
        if(!rst_n) begin
          RX_sync_0 <= 1;
         RX_sync_1 <= 1;
        end  
        else begin
        RX_sync_0 <= RX;
        RX_sync_1 <= RX_sync_0;
        end
   
end
/////////////////////////
// Baud Rate Generator //
/////////////////////////
always_ff @(posedge clk) begin
    if (start)
        baud_cnt <=  (BAUD_TIME*3)/2;
    else if (shift)
	    baud_cnt <=  BAUD_MAX ;
    else if (receiving && baud_cnt == 0 )
        baud_cnt <= BAUD_MAX;
    else if(receiving && baud_cnt != 0 ) 
            baud_cnt <= baud_cnt - 1;
	
end

// Shift occurs when the counter hits zero
assign shift = (baud_cnt == 0) ? 1'b1 : 1'b0;

/////////////////
// Bit Counter //
/////////////////
always_ff @(posedge clk) begin
    if (start)
        bit_cnt <= '0;
    else if (shift )
        bit_cnt <= bit_cnt + 1;
end

////////////////////////////////////////
// Shift Register & Output Assignment //
////////////////////////////////////////
always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) 
        rx_shift_reg <= '0;
     else if (shift )
        rx_shift_reg <= {RX_sync_1, rx_shift_reg[8:1]};
end

// After shift, rx_shift register holds stop bit
assign rx_data = rx_shift_reg[7:0];


// State Transition Logic
always_ff @(posedge clk or negedge rst_n ) begin
    if (!rst_n)
        curr_state <= IDLE;
    else
        curr_state <= nxt_state;
end

// Next State Logic
always_comb begin
    // Default assignments to prevent inferred latches
    nxt_state = curr_state;
    start     = 0;
    receiving = 0;
    set_rdy   = 0;

    case (curr_state)

        IDLE: begin
            // Wait for a falling edge on the RX line
            if (~RX_sync_1) begin
                start     = 1;
                receiving = 1;
                nxt_state = RECEIVING;
            end
        end

        RECEIVING: begin
            receiving = 1;
            // When the last bit is received, set ready signal and return to IDLE
            if (bit_cnt == 4'd9) begin
                set_rdy   = 1;
                nxt_state = IDLE;
            end
        end

    endcase
end

// SR Flip-Flop for Ready Signal
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdy <= 0;
	end 
    else if (S)
        rdy <= 1;
    else if (R)
        rdy <= 0;
    else if (clr_rdy)
        rdy <= 0;
end

endmodule