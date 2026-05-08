import test_suite::*;

module MazeRunner_cal_tb();

  reg clk,RST_n;
  reg send_cmd;					// assert to send command to MazeRunner_tb
  reg [15:0] cmd;				// 16-bit command to send
  reg [11:0] batt;				// battery voltage 0xD80 is nominal
  
  logic cmd_sent;				
  logic resp_rdy;				// MazeRunner has sent a pos acknowledge
  logic [7:0] resp;				// resp byte from MazeRunner (hopefully 0xA5)
  logic hall_n;					// magnet found?
  
  /////////////////////////////////////////////////////////////////////////
  // Signals interconnecting MazeRunner to RunnerPhysics and RemoteComm //
  ///////////////////////////////////////////////////////////////////////
  wire TX_RX,RX_TX;
  wire INRT_SS_n,INRT_SCLK,INRT_MOSI,INRT_MISO,INRT_INT;
  logic lftPWM1,lftPWM2,rghtPWM1,rghtPWM2;
  wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;
  wire IR_lft_en,IR_cntr_en,IR_rght_en;  
  wire piezo;
logic passing; // whether the testbench has passed or failed, we assume passing until we find a failure case      
logic snd_cmd;

  logic [19:0] heading_prev; 

  //////////////////////
  // Instantiate DUT //
  ////////////////////
  MazeRunner iDUT(.clk(clk),.RST_n(RST_n),.INRT_SS_n(INRT_SS_n),.INRT_SCLK(INRT_SCLK),
                  .INRT_MOSI(INRT_MOSI),.INRT_MISO(INRT_MISO),.INRT_INT(INRT_INT),
				  .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
				  .A2D_MISO(A2D_MISO),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
				  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),.RX(RX_TX),.TX(TX_RX),
				  .hall_n(hall_n),.piezo(piezo),.piezo_n(),.IR_lft_en(IR_lft_en),
				  .IR_rght_en(IR_rght_en),.IR_cntr_en(IR_cntr_en),.LED());
	
  ///////////////////////////////////////////////////////////////////////////////////////
  // Instantiate RemoteComm which models bluetooth module receiving & forwarding cmds //
  /////////////////////////////////////////////////////////////////////////////////////
  RemoteComm iCMD(.clk(clk), .rst_n(RST_n), .RX(TX_RX), .TX(RX_TX), .cmd(cmd), .snd_cmd(snd_cmd),
               .cmd_snt(cmd_sent), .resp_rdy(resp_rdy), .resp(resp));
			   
  ///////////////////////////////////////////////////
  // Instantiate physical model of robot and maze //
  /////////////////////////////////////////////////
  RunnerPhysics iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(INRT_SS_n),.SCLK(INRT_SCLK),.MISO(INRT_MISO),
                      .MOSI(INRT_MOSI),.INT(INRT_INT),.lftPWM1(lftPWM1),.lftPWM2(lftPWM2),
					  .rghtPWM1(rghtPWM1),.rghtPWM2(rghtPWM2),
                     .IR_lft_en(IR_lft_en),.IR_cntr_en(IR_cntr_en),.IR_rght_en(IR_rght_en),
					 .A2D_SS_n(A2D_SS_n),.A2D_SCLK(A2D_SCLK),.A2D_MOSI(A2D_MOSI),
					 .A2D_MISO(A2D_MISO),.hall_n(hall_n),.batt(batt));


////////////////////////////////////////////////////////////////////////
// This testbench simply tests power-up behavior of the robot
////////////////////////////////////////////////////////////////////////

					 
  initial begin

    passing = 1;
    send_cmd = 0;
    // nominal battery voltage, should not be low enough to trigger low batt behavior 
    batt = 12'hFF0; // below 8'hD90 robot triggers piezo
    clk = 0;
    RST_n = 0;

    // DO NOT send any commands, we want to 
    // make sure that nothing happens without being commanded
   
    // wait to deassert reset
    @(posedge clk);
    @(negedge clk);

    RST_n = 1;

    repeat(3) @(negedge clk);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Robot should automatically setup the NEMO module upon reset. Check that NEMO is in fact 
    // setup on reset.
    /////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin : timeout1 
        repeat(1000000) @(negedge clk);
        $error("ERROR: NEMO was never setup");
        passing = 0;
        $stop;
      end
      begin
        @(posedge iPHYS.iNEMO.NEMO_setup);
        $display("GOOD: NEMO is setup");
        disable timeout1;
      end
    join

    /////////////////////////////////////////////////////////////////////////////////////////////
    // We know that NEMO is setup, command the robot to calibrate the inertial interface,
    // and wait for it to respond.
    /////////////////////////////////////////////////////////////////////////////////////////////
    
    cmd_calibrate(snd_cmd, cmd, clk);
    fork
      begin : timeout2 
        repeat(100000000) @(negedge clk);
        $error("ERROR: Timeout, calibration was not performed successfullly");
        passing = 0;
        $stop;
      end
      // resp_rdy will come later (spi takes time to transmit)
      begin
        @(posedge iDUT.cal_done);
        @(posedge resp_rdy);
        $display("GOOD: Calibration complete");
        disable timeout2;
      end
    join

    ////////////////////////////////////////////////////////////////////////////
    // set battery low enough that the piezo should trigger
    // wait to see if the piezo triggers.
    ////////////////////////////////////////////////////////////////////////////
    batt = 12'hCC0;

    fork
      begin : timeout3 
        repeat(100000000) @(negedge clk);
        $error("ERROR: Timeout, piezo did not start when battery low");
        passing = 0;
        $stop;
      end
      // see if piezo triggers
      begin
        @(negedge piezo);
        $display("GOOD: Piezo rang when battery low");
        disable timeout3;
      end
    join


    // Keep running
    repeat(10000000) @(negedge clk);

    // should not move outside of starting square
    passing = passing & inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd3);
    
    ///////////////////////////////////////////////////////////////////////////
    // Tests complete - check if we passed
    ///////////////////////////////////////////////////////////////////////////
    verify_passed(passing);
    $stop();

  end




  always
    #5 clk = ~clk;
	
endmodule

