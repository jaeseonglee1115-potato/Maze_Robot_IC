/////////////////////////////////////////////////////////////////////////
// This module implements the maze solving logic using state machines //
// It handles whether the robot does a left, right, or U-turn based on //
// the open paths, and moves with IDLE state                           //
/////////////////////////////////////////////////////////////////////////
module maze_solve(
    input logic clk, rst_n,
    input logic cmd_md,
    input logic cmd0,
    input logic lft_opn, rght_opn,
    input logic mv_cmplt,
    input logic sol_cmplt,
    output logic strt_hdng,
    output logic [11:0] dsrd_hdng,
    output logic strt_mv,
    output logic stp_lft, stp_rght
);

// Logic declarations
reg [11:0] dsrd_hdng_reg;
logic turn_lft, turn_rght, turn_180;

// State machine declarations
typedef enum logic [2:0]{
    IDLE,
    MOVE,
    CHECK,
    LEFT_TURN,
    RIGHT_TURN,
    U_TURN
} state_t;

state_t state, next_state;

// Assigning the desired heading based on the current heading and turn direction 
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        dsrd_hdng_reg <= 12'h000;
    else if(turn_lft) begin
    // Left turn 90 degree
        case(dsrd_hdng_reg)
            12'h000: dsrd_hdng_reg <= 12'h3FF; // North to West
            12'h3FF: dsrd_hdng_reg <= 12'h7FF; // West to South
            12'h7FF: dsrd_hdng_reg <= 12'hC00; // South to East
            12'hC00: dsrd_hdng_reg <= 12'h000; // East to North
            default: dsrd_hdng_reg <= 12'h000;

        endcase
        
    // Right turn 90 degree 
    end else if (turn_rght) begin
        case(dsrd_hdng_reg)
            12'h000: dsrd_hdng_reg <= 12'hC00; // North to East
            12'hC00: dsrd_hdng_reg <= 12'h7FF; // East to South
            12'h7FF: dsrd_hdng_reg <= 12'h3FF; // South to West
            12'h3FF: dsrd_hdng_reg <= 12'h000; // West to North
            default: dsrd_hdng_reg <= 12'h000;

        endcase
    
    // U-turn values for each heading
    end else if (turn_180) begin
        case(dsrd_hdng_reg)
            12'h000: dsrd_hdng_reg <= 12'h7FF; // North to South
            12'h3FF: dsrd_hdng_reg <= 12'hC00; // East to West
            12'h7FF: dsrd_hdng_reg <= 12'h000; // South to North
            12'hC00: dsrd_hdng_reg <= 12'h3FF; // West to East
            default: dsrd_hdng_reg <= 12'h000;
        endcase
        
    end
    
end

// State transition logic
assign dsrd_hdng = dsrd_hdng_reg;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end
// Next state logic
always_comb begin
    // Default values for outputs and next state
    next_state = state;
    strt_hdng = 0;

    strt_mv = 0;
    stp_lft = cmd0;
    stp_rght = ~cmd0;

    turn_lft = 0;
    turn_rght = 0;
    turn_180 = 0;
    

    case(state)
        // Ideally, the robot does nothing when command is not given
        IDLE: begin
            if(!cmd_md) next_state = MOVE;
        end

        // Moves forward until the move complt is asserted
        MOVE: begin
            strt_mv = 1;
            if(mv_cmplt) next_state = CHECK;
        end
        
        // Checks the open paths and decides how much it should turn.
        CHECK: begin
            // If the maze is solved aka got the magnet, do nothing
            if(sol_cmplt) next_state = IDLE;
            else if(cmd0)begin
                strt_hdng = 1;
                if(lft_opn) begin
                    turn_lft = 1;
                    next_state = LEFT_TURN;
                end 
                else if(rght_opn) begin
                    turn_rght = 1;
                    next_state = RIGHT_TURN;
                    
                end 
                else begin
                    turn_180 = 1;   
                    next_state = U_TURN;
                end 
            end
            else begin
                strt_hdng = 1;
                if(rght_opn) begin
                    turn_rght = 1;
                    next_state = RIGHT_TURN;
                end
                else if(lft_opn) begin
                    turn_lft = 1;
                    next_state = LEFT_TURN;
                end
                else begin
                    turn_180 = 1;
                    next_state = U_TURN;
                end
            end
        end

        // Tells and waits robot to turn left
        LEFT_TURN: begin
            if(mv_cmplt) next_state = MOVE;
        end

        // Tells and waits robot to turn right
        RIGHT_TURN: begin
            if(mv_cmplt) next_state = MOVE;
        end

        // Tells and waits robot to do a U-turn
        U_TURN: begin
            if(mv_cmplt) next_state = MOVE;
        end
        default : 
            next_state = IDLE;

    endcase
end

endmodule