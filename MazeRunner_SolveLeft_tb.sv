
import test_suite::*;

module MazeRunner_solvemaze_lft_tb();

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

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // The robot should automatically configure NEMO, wait for NEMO to assert that it is setup.
    ///////////////////////////////////////////////////////////////////////////////////////////////

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

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // NEMO is now setup, command the robot to calibrate and wait for it to complete
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cmd_calibrate(snd_cmd, cmd, clk); 
    
    fork
      begin : timeout2 
        repeat(100000000) @(negedge clk);
        $error("ERROR: Timeout, calibration was not performed successfullly");
        passing = 0;
        $stop;
      end
      begin
        @(posedge iDUT.cal_done);
        @(posedge resp_rdy);
        $display("GOOD: Calibration Complete. Robot is ready to move.");
        disable timeout2;
      end
    join

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // We are ready to solve the maze. Command the robot and track where it goes to make sure
    // its behavior is correct.
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cmd_solve_lft(snd_cmd, cmd, clk);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // robot should move from (3, 3) to (3, 2) and point south
    ///////////////////////////////////////////////////////////////////////////////////////////////

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

    // CHECK THAT IT IS HEADING IN RIGHT DIRECTION?

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // robot should move from (3, 2) to (3, 1) and point south
    ///////////////////////////////////////////////////////////////////////////////////////////////

    fork    
      begin  : timeout4
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout moving to (3,1)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd1))
        $display("GOOD: Reached physical coordinate (3,1)");
        disable timeout4;
      end
    join_any 

    // we must be facing south to move south
    passing = passing & check_south(iPHYS.heading_robot);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // robot should move from (3, 1) to (3, 0)
    ///////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout reaching (3,0)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd0));
        $display("GOOD: robot travelled to (3,0)");
      end
    join_any 


    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Robot should turn around here, and nead north from (3, 0) to (3, 1)
    ///////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout reaching (3,1)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd1));
        $display("GOOD: robot travelled to (3,1)");
      end
    join_any 

    // we must be facing north to move north
    passing = passing & check_north(iPHYS.heading_robot);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // The robot should travel from (3, 1) to (3, 2)
    ///////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout reaching (3,2)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd3, 2'd2));
        $display("GOOD: robot travelled to (3,2)");
      end
    join_any 

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // The robot should travel from (3, 2) to (2, 2)
    ///////////////////////////////////////////////////////////////////////////////////////////////

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout reaching (2,2)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd2, 2'd2));
        $display("GOOD: robot travelled to (2,2)");
      end
    join_any 

    // we must be facing west to move west
    passing = passing & check_west(iPHYS.heading_robot);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // The robot should travel from (3, 2) to (1, 2)
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    passing = passing & check_west(iPHYS.heading_robot);

    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout reaching (2,2)");
        passing = 0;
        $stop;
      end
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd1, 2'd2));
        $display("GOOD: robot travelled to (1,2)");
      end
    join_any 

    // we must be facing west to move west
    passing = passing & check_west(iPHYS.heading_robot);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Robot is in the same square as magnet, it should detect it soon. 
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    fork
      begin
        repeat(10000000) @(negedge clk);
        $error("ERROR: Timeout detecting magnet in square");
        passing = 0;
        $stop;
      end
      
      begin
        wait(inside_square(iPHYS.xx, iPHYS.yy, 2'd2, 2'd2) ||
              inside_square(iPHYS.xx, iPHYS.yy, 2'd1, 2'd1))
        $error("ERROR: robot moved out of square without detecting magnet");
        passing = 0;
        $stop;
      end

      begin
        wait(iDUT.sol_cmplt);
        @(negedge resp_rdy);
        $display("GOOD: detected magnet");
      end
    join_any 

    // we must be facing west to move west
    passing = passing & check_west(iPHYS.heading_robot);

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

    
    verify_passed(passing);
    $stop();

  end

  always
    #5 clk = ~clk;
	
endmodule


