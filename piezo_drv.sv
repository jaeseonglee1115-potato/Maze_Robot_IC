module piezo_drv #(parameter FAST_SIM = 0) (

    input clk, rst_n, batt_low, fanfare,
    output piezo, piezo_n

);

// FSM states for playing the notes
typedef enum logic [3:0] {IDLE, G6, C7, E7_0, G6_BAT, C7_BAT, 
                            E7_0_BAT, G7_0, E7_1, G7_1} state_p;

state_p state, nxt_state;

logic [24:0] duration_cnt, duration;
logic [14:0] frequency_cnt, frequency;
logic strt_note, done_note, inc_en;

// the outputs are not synchonized with FF's because frequency_cnt[14] is a 
// FF and is in synchronously loaded. The inverter will only add minimal output delay
// (on the order of nanoseconds)
assign piezo = frequency_cnt[13] | frequency_cnt[14];
assign piezo_n = ~(frequency_cnt[13] | frequency_cnt[14]);

always_comb begin
    
    // defaults
    frequency = 15'hxxxx;
    duration = 25'hxxxx;
    strt_note = 1'b0;
    inc_en = 1'b1;
    nxt_state = state;

    case(state)
        IDLE: begin
            inc_en = 1'b0;
            // state playing first note G6 for 2^23 cycles
            // batt_low should play first
            if(batt_low) begin
                nxt_state = G6_BAT;
                strt_note = 1'b1;
                duration = 25'h0800000;
                frequency = 15'd31888;
            // fanfare should go if batt_low is not asserted
            end else if(fanfare) begin
                strt_note = 1'b1;
                duration = 25'h0800000;
                frequency = 15'd31888;
                nxt_state = G6;
            end
        end
        G6: begin
            frequency = 15'd31888;
            // first note is done now play C7 for 2^23 cycles
            if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd23889;
                duration = 25'h0800000;
                nxt_state = C7;
            end
        end
        C7:begin
            frequency = 15'd23889;
            // play E7 for 2^23 cycles after C7
            if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd18961;
                duration = 25'h0800000;
                nxt_state = E7_0;
            end
        end
        E7_0: begin
            frequency = 15'd18961;
            // play G7 for 2^23+2^22cycles after E7
            if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd15944;
                duration = 25'h0A00000;
                nxt_state = G7_0;
            end
        end
        
        ////////////////////////////////////////////////
        // These states for playing batt_low sequence
        ////////////////////////////////////////////////
        G6_BAT:begin 
            frequency = 15'd31888;
            // first note is done now play C7 for 2^23 cycles
            if(!batt_low) begin
                nxt_state = IDLE;
            end else if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd23889;
                duration = 25'h0800000;
                nxt_state = C7_BAT;
            end
        end

        C7_BAT:begin
            frequency = 15'd23889;
            // play E7 for 2^23 cycles after C7
            if(!batt_low) begin
                nxt_state = IDLE;
            end else if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd18961;
                duration = 25'h0800000;
                nxt_state = E7_0_BAT;
            end
        end

        E7_0_BAT: begin
            frequency = 15'd18961;
            // play G7 for 2^23+2^22cycles after E7
            if(!batt_low) begin
                nxt_state = IDLE;
            end else if(done_note) begin
                nxt_state = IDLE;
            end
        end
        /////////////////////////////////////////////
        // fanfare will continue from here
        /////////////////////////////////////////////
        G7_0: begin
            frequency = 15'd15944;
            // if batt_low then we only play the first three notes. 
            // will play G7 for at least single clock cycle if batt_low.
            // this doesn't really matter because it will only play for a few nanoseconds.
            if(batt_low) 
                nxt_state = IDLE;
            // play E7 for 2^22 + 2^22 cycles after EG
            else if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd18961;
                duration = 25'h0400000;
                nxt_state = E7_1;
            end
        end
        E7_1: begin
            frequency = 15'd18961;
            // play G7 for 2^24 cycles after E7
            if(done_note) begin
                strt_note = 1'b1;
                frequency = 15'd15944;
                duration = 25'h1000000;
                nxt_state = G7_1;
            end
        end
        G7_1: begin
            frequency = 15'd15944;
            // we are done with fanfare return to IDLE
            if(done_note)
                nxt_state = IDLE;
        end
        // if in an unknown state, go back to idle for safety
        default: 
            nxt_state = IDLE;
    endcase
end

// the note is done when the duration is all 0's
assign done_note = ~|duration_cnt;


// duration counter resets every time a note is started
// keeps track of how long a note has been played
generate
    // if we are simulating, shorted duration by factor 1/16
    if (FAST_SIM) begin
        always_ff @(posedge clk) 
            if(strt_note)
                duration_cnt <= duration;
            else if(inc_en && !strt_note)
                duration_cnt <= duration_cnt - 24'h000010;

    // we are making this in hardware, duration should be nominal.
    end else begin
        always_ff @(posedge clk) 
            if(strt_note)
                duration_cnt <= duration;
            else if(inc_en && !strt_note)
                duration_cnt <= duration_cnt - 24'h000001;
    end
endgenerate

// frequency counter for creating notes
always_ff @(posedge clk)
    if(strt_note || ~|frequency_cnt)
        frequency_cnt <= frequency;
        // if we are playing a note (inc_en) then divide the input frequency to make
        // the note
    else if(inc_en && !strt_note)
        frequency_cnt <= frequency_cnt - 15'h0001;

// FSM state flops keeps track of the note being played
always_ff @(posedge clk, negedge rst_n) 
    if(!rst_n)
        state <= IDLE;
    else 
        state <= nxt_state;
endmodule
