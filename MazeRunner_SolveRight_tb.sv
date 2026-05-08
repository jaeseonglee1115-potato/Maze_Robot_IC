
import test_suite::*;

module MazeRunner_solvemaze_tb();

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
  logic [3:0] num_tests;

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
   
    // wait to deassert reset
    @(posedge clk);
    @(negedge clk);
    
    RST_n = 1;
     
    repeat (3) @(posedge clk);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Robot should automatically setup the NEMO module upon reset. Check that NEMO is in fact 
    // setup on reset.
    /////////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin : timeout1 
        repeat(1000000) @(negedge clk);
        $error("ERROR: NEMO was never setup");
        passing = 0;
        $stop;
      end
      begin
        @(posedge iPHYS.iNEMO.NEMO_setup);
        $display("GOOD: NEMO is setup. Robot is ready to calibrate.");
        disable timeout1;
      end
    join

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // We know that NEMO is setup, command the robot to calibrate the inertial interface,
    // and wait for it to respond. (do not check inside, we did this already in cal_tb)
    /////////////////////////////////////////////////////////////////////////////////////////////////

    cmd_calibrate(snd_cmd, cmd, clk); 
    
    fork
      begin : timeout2 
        repeat(100000000) @(negedge clk);
        $error("ERROR: Timeout, calibration was not performed successfullly");
        passing = 0;
        $stop;
      end
      begin
        @(posedge resp_rdy);
        $display("GOOD: Calibration Complete. Robot is ready to move.");
        disable timeout2;
      end
    join

    // COMMAND SOLVE RGHT AFFINITY
    cmd_solve_rght(snd_cmd, cmd, clk);
  
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // we know where we expect the robot to go given the rght affinity, hence we will check that 
    // it is taking a path dictated by right affinity.
    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Wait for the robot to navigate to grid (3,2)
    fork
      begin : timeout3
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout moving to (3,2)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd2));
        $display("GOOD: Reached physical coordinate (3,2)");
        disable timeout3;
      end
    join

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Check where it moves next, is should move to [2,2] anything else counts as failure
    // Also, change battery voltage to lower, yet valid voltage
    /////////////////////////////////////////////////////////////////////////////////////////////////
    batt = 12'hEF0; 
    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout moving to 2,2");
        passing = 0;
        $stop;
      end
      
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd3) ||
              inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd1))
        $error("ERROR: robot did not move to expected square (2,2)");
        passing = 0;
        $stop;
      end

      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd2, 2'd2));
        $display("GOOD: robot travelled to (2,2)");
      end
    join_any 

    // should be pointing west
    passing = passing & check_west(iPHYS.heading_robot);

    // set battery even lower (yet still valid voltage)
    batt = 12'hE00; 

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Robot should solve maze after entering next cell, [1, 2]
    /////////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout reaching 1,2 ");
        passing = 0;
        $stop;
      end
      
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd2))
        $error("ERROR: robot did not move to expected square (1,2)");
        passing = 0;
        $stop;
      end

      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd1, 2'd2));
        $display("GOOD: robot travelled to (1,2)");
      end
    join_any 

    // robot should still pointing west
    passing = passing & check_west(iPHYS.heading_robot);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Robot should now find the magnet and complete the maze
    /////////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout detecting magnet in square (1,1)");
        passing = 0;
        $stop;
      end
      
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd2, 2'd2) ||
              inside_square(iPHYS.xx, iPHYS.yy, 2'd1, 2'd1))
        $error("ERROR: robot moved out of square without detecting magnet (1,1)");
        passing = 0;
        $stop;
      end

      begin
        @(posedge resp_rdy);
        $display("GOOD: detected magnet");
      end
    join_any 

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Check that piezo goes low, we already tested piezo in hardward, so rigorous tests aren't 
    // really too beneficial
    /////////////////////////////////////////////////////////////////////////////////////////////////
    fork
      begin : timeout12
        repeat(1000000) @(negedge clk);
        $error("ERROR: Timeout waiting for piezo to activate");
        passing = 0;
        $stop;
      end
      begin
        wait(!piezo);
        $display("GOOD: piezo has started chiming");
        disable timeout12;
      end
    join

    // Make sure that robot doesn't move after finishing maze
    fork
      begin : timeout13
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd2, 2'd2) ||
              inside_square(iPHYS.xx, iPHYS.yy, 2'd1, 2'd1))
        $error("ERROR: robot moved out of square after detecting magnet");
        passing = 0;
        $stop;
      end

      begin
        repeat(1000000) @(posedge clk);
        $display("GOOD: robot did not move after finding magnet");
        disable timeout13;
      end
    join

    // robot should not turn once it has found the magnet
    passing = passing & check_west(iPHYS.heading_robot);
    
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Tests complete - check if we passed
    /////////////////////////////////////////////////////////////////////////////////////////////////
    verify_passed(passing);
    $stop();

  end

  always
    #5 clk = ~clk;
	
endmodule


