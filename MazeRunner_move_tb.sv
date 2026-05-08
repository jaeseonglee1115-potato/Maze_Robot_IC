import test_suite::*;

module MazeRunner_move_tb();

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
  logic outport;
  logic prev_omega;
  logic prev_err;

  logic passing_timeout;

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
  batt = 12'hFF0; // nominal battery voltage, should not be low enough to trigger low batt behavior
  clk = 0;
  RST_n = 0;
  
  // wait to deassert reset
  @(posedge clk);
  @(negedge clk);

  RST_n = 1;

  repeat(3) @(negedge clk);
  ///////////////////////////////////////////////////////////////////////
  // Wait for NEMO setup , basic test
  ///////////////////////////////////////////////////////////////////////   
  fork: timeout1 
    begin 
      repeat(1000000) @(negedge clk);
      $error("ERROR: NEMO was never setup");
      passing = 0;
      $stop;
    end
    begin
      @(posedge iPHYS.iNEMO.NEMO_setup);
      $display("GOOD: NEMO is setup");
      
    end
  join_any
  disable timeout1;

  //////////////////////////////////////////////////////////////////////
  // start calibration, wait 10  MILLION clk cycles to calibrate
  //////////////////////////////////////////////////////////////////////

  cmd_calibrate(snd_cmd, cmd, clk);
  fork: timeout2  
    begin 
      repeat(100000000) @(negedge clk);
      $error("ERROR: Timeout, calibration was not performed successfullly");
      passing = 0;
      $stop;
    end
    // resp_rdy will come later (spi takes time to transmit)
    begin
      @(posedge iDUT.cal_done);
      @(posedge resp_rdy);
      $display("GOOD: calibration complete");
      disable timeout2;

    end
  join_any

  //////////////////////////////////////////////////////////////////////
  // move it into a wall It should eventually stop in the square it 
  // started in.
  //////////////////////////////////////////////////////////////////////

  cmd_move_rght(snd_cmd, cmd, clk);
    fork: timeout_wall_fail
      begin  
        repeat(10000000) @(negedge clk);
        $error("ERROR: Turning a sequence not performed properly");
        passing = 0;
        $stop;
      end
      // resp_rdy will come later (spi takes time to transmit)
      begin
        @(posedge iDUT.mv_cmplt);
        @(posedge resp_rdy);      
        $display("GOOD: robot has finished moving");
        disable timeout_wall_fail;
      end
    join_any

  // should not exit square, wall is ahead!
  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,3);

  //////////////////////////////////////////////////////////////////////
  // Lets turn south so we can start getting somewhere
  //////////////////////////////////////////////////////////////////////

  cmd_south(snd_cmd, cmd, clk);
  fork: timeout3
    begin  
      repeat(10000000) @(negedge clk);
      $error("ERROR: Turning a sequence not performed properly");
      passing = 0;
      $stop;
    end
    // resp_rdy will come later (spi takes time to transmit)
    begin
      @(posedge iDUT.mv_cmplt);
      @(posedge resp_rdy);      
      $display("GOOD: robot has finished moving");
      disable timeout3;
    end
  join_any

    passing = passing & check_south(iPHYS.heading_robot);
  
  // robot should not move 
  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,3);

  ///////////////////////////////////////////////////////////
  // move with right affnity, we expet to move to 3,2 from 3,3 (starting position) 
  ///////////////////////////////////////////////////////////////////////


  // THIS SHOULD WAIT FOR WHEELS TO RAMP UP, THEN WAIT FOR MV_CMPLT AND RESPONSE FROM UART
  cmd_move_rght(snd_cmd, cmd, clk);

  fork   : timeout4

    begin
      repeat(10000000) @(negedge clk);
      $error("1 move_cmplt did not work ");
        passing =0;
    end

    begin
      @(posedge iDUT.mv_cmplt);
      $display("GOOD: move complete");
    end

  join_any 
  disable timeout4;

    // does omega sum ramp up ?
  if(!(iPHYS.omega_sum > 0))begin
    $error("ERROR: %d %d", iPHYS.omega_sum , prev_omega );
    passing  = 0;
    $stop();
  end
  else begin
      $display("GOOD: omega, sum is positive (mving fwrd)");
  end

  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,2);

 //////////////////////////////////////////////////////////////////////////////////////////
 //  now we want to move it west, relative to facing south, so that would be objective east 
 //////////////////////////////////////////////////////////////////////////////////////////

  cmd_west(snd_cmd, cmd, clk);
    fork   : timeout5

    begin
      repeat(10000000) @(negedge clk);
      $display("ERROR: timeout on mv_cmplt");
      passing =0;
    end

    begin
        @(posedge iDUT.mv_cmplt);
        $display("GOOD: 2 move_cmplt did assert as expected");
    end

  join_any
  disable timeout5;

  passing = passing & check_west(iPHYS.heading_robot);

  //////////////////////////////////////////////////////////////////////////////////////////
  // move with RIGHT affinity, and we hope to move forward  
  //////////////////////////////////////////////////////////////////////////////////////////

  cmd_move_rght(snd_cmd, cmd, clk);
  fork   : timeout6

    begin
      repeat(10000000) @(negedge clk);
      $display("ERROR: timeout on mv_cmplt");
      passing =0;
    end

    begin
        @(posedge iDUT.mv_cmplt);
        $display("GOOD: move complete");
    end

  join_any 
  disable timeout6;

  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 1,2);

  //////////////////////////////////////////////////////////////////////////////////////////
  // move with RIGHT affinity, and we hope to move forward  
  //////////////////////////////////////////////////////////////////////////////////////////
    
  cmd_east(snd_cmd, cmd, clk);
  fork   : timeout7

    begin
      repeat(10000000) @(negedge clk);
      $display("ERROR: timeout on mv_cmplt");
      passing =0;
    end

    begin
      @(posedge iDUT.mv_cmplt);
      $display("GOOD: move complete");
    end
  join_any
  disable timeout7;

  passing = passing & check_east(iPHYS.heading_robot);

  //////////////////////////////////////////////////////////////////////////////////////////
  // move with LEFT affinity, and we hope to move forward  
  //////////////////////////////////////////////////////////////////////////////////////////
  

  cmd_move_lft(snd_cmd, cmd, clk);
  fork   : timeout8

    begin
      repeat(10000000) @(negedge clk);
      $display("move_cmplt did not work ");
      passing =0;
    end

    begin
      @(posedge iDUT.mv_cmplt);
      $display("GOOD: move complete");
    end

  join_any 
  disable timeout8;

  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,2);
  

  //////////////////////////////////////////////////////////////////////////////////////////
  // orient southward, and we should then move forward  to 3,0
  //////////////////////////////////////////////////////////////////////////////////////////
  cmd_south(snd_cmd, cmd, clk);
  fork   : timeout9

    begin
      repeat(10000000) @(negedge clk);
      $display("move_cmplt did not work ");
      passing =0;
    end

    begin
      @(posedge iDUT.mv_cmplt);
      $display("GOOD: move complete");
    end

  join_any 
  disable timeout9;

  cmd_move_lft(snd_cmd, cmd, clk);
  fork   : timeout10

    begin
      repeat(10000000) @(negedge clk);
      $display("move_cmplt did not work ");
      passing =0;
    end

    begin
      @(posedge iDUT.mv_cmplt);
      @(posedge resp_rdy);
      $display("GOOD: move complete");
    end

  join_any 
  disable timeout10;

  
  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,0);

  //////////////////////////////////////////////////////////////////////////////////////////
  // orient north, and we should then move forward  to 3,2
  //////////////////////////////////////////////////////////////////////////////////////////
  cmd_north(snd_cmd, cmd, clk);
  
  // make sure that the reponse is timely
  command_timeout(passing_timeout, resp, resp_rdy, clk);
  passing = passing & passing_timeout;

  // give move command
  cmd_move_lft(snd_cmd, cmd, clk);

  // check direction
  passing = passing & check_north(iPHYS.heading_robot);

  // make sure that the reponse is timely
  command_timeout(passing_timeout, resp, resp_rdy, clk);
  passing = passing & passing_timeout;

  // check we are in expected square
  passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,2);

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // wait for a while to just check it doesnt move to any of the neighbouring squres (3 possible)
  // currently in 3, 2
  ////////////////////////////////////////////////////////////////////////////////////////////////

  fork : timeout13

    begin
      repeat(10000000) @(negedge clk);
      $display("GOOD: robot did not move when not commanded");
    end
    begin
      wait(inside_square(iPHYS.xx, iPHYS.yy, 2,3) || inside_square(iPHYS.xx, iPHYS.yy, 3,1) ||
            inside_square(iPHYS.xx, iPHYS.yy, 3,3));
      passing =0;
      $display("ERROR: robot moved when not commanded");
      $stop;
    end
  join_any
  disable timeout13;

  // evaluate test results
  verify_passed(passing);

  end

  always
    #5 clk = ~clk;
	
endmodule


