//////////////////////////////////////////////////////
// Interfaces with ST 6-axis inertial sensor.  In  //
// this application we only use Z-axis gyro for   //
// heading of mazeRunner.  Fusion correction     //
// comes from IR_Dtrm when en_fusion is high.   //
/////////////////////////////////////////////////
module inert_intf #(parameter FAST_SIM = 0) // speed up simulation reduce cal samples.
                (clk,rst_n,strt_cal,cal_done,heading,rdy,IR_Dtrm,
                SS_n,SCLK,MOSI,MISO,INT,moving,en_fusion);

input clk, rst_n;
input MISO;							// SPI input from inertial sensor
input INT;							// goes high when measurement ready
input strt_cal;						// initiate claibration of yaw readings
input moving;							// Only integrate yaw when going
input en_fusion;						// do fusion corr only when forward at decent clip
input [8:0] IR_Dtrm;					// derivative term of IR sensors (used for fusion)

output cal_done;				// pulses high for 1 clock when calibration done
output signed [11:0] heading;	// heading of robot.  000 = Orig dir 3FF = 90 CCW 7FF = 180 CCW
output rdy;					// goes high for 1 clock when new outputs ready (from inertial_integrator)
output SS_n,SCLK,MOSI;		// SPI outputs


////////////////////////////////////////////
// Declare any needed internal registers //
//////////////////////////////////////////



logic [7:0]inert_low, inert_high;
logic [1:0]INT_ff;


//////////////////////////////////////
// Outputs of SM are of type logic //
////////////////////////////////////

logic C_Y_H, C_Y_L;

//////////////////////////////////////////////////////////////
// Declare any needed internal signals that connect blocks //
////////////////////////////////////////////////////////////
wire done;
wire [15:0] inert_data;		// Data back from inertial sensor (only lower 8-bits used)
wire signed [15:0] yaw_rt;
logic [15:0] timer;
logic [15:0] cmd;
logic wrt, vld;

///////////////////////////////////////
// Create enumerated type for state //
/////////////////////////////////////
typedef enum logic [2:0] { INIT1, INIT2, INIT3, INIT4, IDLE, INERT_L, INERT_H, VLD} state_i;
state_i state, nxt_state;

////////////////////////////////////////////////////////////
// Instantiate SPI monarch for Inertial Sensor interface //
//////////////////////////////////////////////////////////

//////////// USED SPI_main instead of SPI_mnrch /////////////
SPI_main iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),
                .MISO(MISO),.MOSI(MOSI),.wrt(wrt),.done(done),
        .rd_data(inert_data),.wt_data(cmd));
        
////////////////////////////////////////////////////////////////////
// Instantiate Angle Engine that takes in angular rate readings  //
// and gaurdrail info and produces a heading reading            //
/////////////////////////////////////////////////////////////////
inertial_integrator #(FAST_SIM) iINT(.clk(clk), .rst_n(rst_n), .strt_cal(strt_cal),
                      .vld(vld),.rdy(rdy),.cal_done(cal_done), .yaw_rt(yaw_rt),.moving(moving),
          .en_fusion(en_fusion),.IR_Dtrm(IR_Dtrm),.heading(heading));

assign yaw_rt = {inert_high, inert_low};

always_comb begin

  C_Y_H = 0;
  C_Y_L = 0;
  wrt = 0;
  vld = 0;
  nxt_state = state;
  cmd = 16'hxxxx;

  case(state)

    // wait for the initialization timer to finish then 
    // enable interrupt
    INIT1: 
      if(&timer) begin
        nxt_state = INIT2;
        wrt = 1;
        cmd = 16'h0D02;
      end
    // wait for previous transmission to finish then send then
    // setup gyro rate
    INIT2:
      if(done) begin
        nxt_state = INIT3;
        wrt = 1;
        cmd = 16'h1160;
      end
    // wait for previous transmission to finish then send then
    // turn on wraparound
    INIT3:
      if(done) begin
        nxt_state = INIT4;
        wrt = 1;
        cmd = 16'h1440;
      end
    // wait for previous transmission to finish then get
    // first yaw values when INT_ff is asserted
    INIT4:
      if(done) begin
        nxt_state = IDLE;
      end
    // If in idle, all initialization has been completed and
    // there is a valid gyro reading in the holding registers.
    IDLE:
      if(INT_ff[1]) begin
        wrt = 1;
        cmd = 16'hA6xx;
        nxt_state = INERT_L;
      end
    // Store yawL then start getting yawH
    INERT_L:
      if(done) begin
        C_Y_L = 1;
        wrt = 1;
        cmd = 16'hA7xx;
        nxt_state = INERT_H;
      end
    // store yawH then go back to IDLE
    INERT_H:
      if(done) begin
        C_Y_H = 1;
        nxt_state = VLD;
      end
    VLD: begin
        vld =1;
        nxt_state = IDLE;
    end
    default : 
        nxt_state = INIT1;
  endcase
end

// create 16-bit timer for initicalization
always_ff @(posedge clk, negedge rst_n)
  if(!rst_n)
    timer <= 16'h0000;
  else
    timer <= timer + 16'h0001;

// double flop INT for metastability
always_ff @(posedge clk)
  INT_ff <= {INT_ff[0], INT};


// register for high byte of inter data
always_ff @(posedge clk)
  if(C_Y_H) 
    inert_high <= inert_data[7:0];

// register for low byte of inter data
always_ff @(posedge clk)
  if(C_Y_L)
    inert_low <= inert_data[7:0];

  // state flops
always_ff @(posedge clk, negedge rst_n)
  if(!rst_n)
    state <= INIT1;
  else 
    state <= nxt_state;

endmodule
  