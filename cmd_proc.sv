module cmd_proc (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [15:0] cmd,
    input  logic        cmd_rdy,
    output logic        clr_cmd_rdy,
    output logic        send_resp,
    output logic        strt_cal,
    input  logic        cal_done,
    output logic        in_cal,
    input  logic        sol_cmplt,
    output logic        strt_hdng,
    output logic        strt_mv,
    output logic        stp_lft,
    output logic        stp_rght,
    output logic [11:0] dsrd_hdng,
    input  logic        mv_cmplt,
    output logic        cmd_md
);

logic in_idle; // additional signal 

// 5 states to control the folow, this will send to PID or other module to get specific response related to move
typedef enum logic [2:0] { IDLE,
                        CALIBRATE,
                        HEADING,
                        MOVEMENT,
                        SOLVE_MAZE
                         } state_t;
state_t state, next_state;

// state transition logic

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

// registered outputs
logic [11:0] dsrd_hdng_reg;     //register for setting desred heading 
logic stp_lft_reg, stp_rght_reg;    // reg for indicating stopping left or right

always_ff @( posedge clk or negedge rst_n ) begin
    if (!rst_n) begin
        dsrd_hdng_reg <= 0;     // reset values
        stp_lft_reg <= 0;
        stp_rght_reg <= 0;

    end else if (in_idle && cmd_rdy) begin
        if(cmd[15:13] == 3'h1) begin
            dsrd_hdng_reg <= cmd[11:0]; // default value for dsrd_hdng
        end
        if (cmd[15:13] == 3'h2) begin
            if(cmd[0]) begin    
                stp_rght_reg <= 1;  // stop right       
                stp_lft_reg <= 0;    
            end else begin
                stp_lft_reg <= 1;  // stop left
                stp_rght_reg <= 0;
            end
        end
    end
end

assign dsrd_hdng = dsrd_hdng_reg;     // use registers for some outputs, avoids glitching!
assign stp_lft = stp_lft_reg;
assign stp_rght = stp_rght_reg;

// next state logic

always_comb begin
    clr_cmd_rdy = 0;
    strt_cal = 0;     // default most signals to zero
    in_cal = 0;
    in_idle =0;
    strt_mv = 0;
    strt_hdng = 0;
    send_resp = 0;

    cmd_md = 1; // defulted to high for most states , except solving maze
    next_state = state;


    case(state)

    IDLE: begin
        in_idle =1'b1;   // indicate we are in idle, internal signal
        if (cmd_rdy) begin
            clr_cmd_rdy = 1;  // assert as command is received
            case(cmd[15:13])      // calibrte if 0 
                3'b000: begin
                    next_state = CALIBRATE;
                    strt_cal = 1;
                end
                3'b001: begin
                    next_state = HEADING; // calibrte if 0 
                    strt_hdng = 1;
                end
                3'b010: begin
                    next_state = MOVEMENT;
                    strt_mv = 1;
                end

                3'b011: begin
                    next_state = SOLVE_MAZE;
                end
                default: next_state = IDLE; 
            endcase
        end
    end

    CALIBRATE: begin
        in_cal = 1;
        if (cal_done) begin    //  wait for calibration
            send_resp = 1;
            next_state = IDLE;   //return back to idle
        end
    end

    HEADING: begin
    //    dsrd_hdng = cmd[11:0];
        if (mv_cmplt) begin
            send_resp = 1;    // if move is complete ,send response baack by indicateing high 
            next_state = IDLE;   // idle
        end
    end

    MOVEMENT: begin
        if (mv_cmplt) begin   // we wait for it to complete moving 
            send_resp = 1;
            next_state = IDLE;
        end
    end
    SOLVE_MAZE: begin
        cmd_md = 0; // set to low for solving maze mode
        if (sol_cmplt) begin   // once solving is complete, indicate send_response
            send_resp = 1;  
            next_state = IDLE;  
        end
    end
    default:
        next_state = IDLE;
endcase
    
end

endmodule