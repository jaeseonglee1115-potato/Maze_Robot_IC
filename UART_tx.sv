module UART_tx (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        trmt,
    input  logic [7:0]  tx_data,
    output logic        TX,
    output logic        tx_done
);

localparam int BAUD_RATE = 19200;
  
localparam int BAUD_TICKS = 50_000_000 / BAUD_RATE; // 2604
localparam int BAUD_MAX   = BAUD_TICKS - 1; // 2603

// lny keep two stated
typedef enum logic {
    IDLE,
    TRANSMITTING
} state_t;

state_t curr_state, nxt_state; //currnt and next  states
  
// signls in the data path itslf
logic [11:0] baud_cnt;
logic [3:0]  bit_cnt;
logic [8:0]  tx_shft_reg;

logic init;  // fr all sigals that are outputs of the state machine
logic shift;
logic transmitting;
logic set_done;

  
// tx register logic, contains datas to be sent, and each clock, it sends the data
always_ff @(posedge clk or negedge rst_n) begin
    // active lo2w set ,  conected to reset 
        if(!rst_n) begin
            tx_shft_reg <= '1; // 
        end 
        else if  (init) begin
            // {stop bit, data[7:0], start bit}
            tx_shft_reg <= {tx_data, 1'b0};
        end
        else if (shift) begin
            tx_shft_reg <= {1'b1, tx_shft_reg[8:1]};
        end
    
end

assign TX = tx_shft_reg[0];

// Baud Counter logic 
always_ff @(posedge clk) begin
    
     if (init | shift)  // always set t zer at bginning of tansmit or shift
        baud_cnt <= 12'd0;
    else if (transmitting)
        baud_cnt <= baud_cnt + 1;  // increment baud count
end

assign shift = (baud_cnt == BAUD_MAX) ;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bit_cnt <= 4'd0;  // set zero on reset 
    else if (init)
        bit_cnt <= 4'd0; // set zero on transmit begin 
    else if (shift)
        bit_cnt <= bit_cnt + 1; // inremnt on shft signal
end

// next state logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        curr_state <= IDLE;  
    else
        curr_state <= nxt_state;
end

always_comb begin
    nxt_state     = curr_state;
    init          = 1'b0;
    transmitting  = 1'b0;   
    set_done      = 1'b0;

    case (curr_state)

        IDLE: begin
            if (trmt) begin
                init      = 1'b1;  // init upon transmision
                nxt_state = TRANSMITTING;
		
            end
        end

        TRANSMITTING: begin
            
		transmitting = 1'b1;
                if (bit_cnt == 4'd9 &&  shift) begin
                    nxt_state = IDLE;  // go bck to whre we started
                    set_done  = 1'b1;  // set done aftr 9 bts transmitted successfully, we're at the 10th bit
                end
		else begin 
			
		 end 
    
        end

    endcase
end


// Register that sets tx_done signal when transmission complete and reset when the transmission starts
// Also sets tx_done not 1 when init is currently in process.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tx_done <= 1'b0;  // upon reset 
    else if (set_done || init)
        tx_done <= set_done & ~init; // just take the value of set-done,whch will most likely by zero 
end

endmodule