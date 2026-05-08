import test_suite::*;

module MazeRunner_turn_tb();

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

  ///// Internal registers for testing purposes??? /////////
  bit passing_timeout;
  bit [1:0]turn_dir; 

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
    // DO NOT send any commands, we want to make sure that nothing happens without being commanded
   
    // wait to deassert reset
    @(posedge clk);
    @(negedge clk);

    RST_n = 1;

    repeat(3) @(negedge clk);

    // wait for NEMO to be setup, fail if it doesn't 

    // wait for NEMO to be setup    
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

    // start calibration
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
        $display("GOOD: calibration complete");
        disable timeout2;
      end
    join

    //////////////////////////////////////////////////////
    // Check turning West
    // We are done calibrating, now just test turning
    //////////////////////////////////////////////////////

    cmd_west(snd_cmd, cmd, clk);

    fork
      begin : timeout3 
        repeat(10000000) @(negedge clk);
        $error("ERROR: Turning a sequence not performed properly");
        passing = 0;
        $stop;
      end
      // resp_rdy will come later (spi takes time to transmit)
      begin
        @(posedge iDUT.mv_cmplt);
        @(posedge resp_rdy);
        $display("GOOD: turn west complete");
        disable timeout3;
      end
    join

    if(iDUT.dsrd_hdng !== WEST) begin
      $error("ERROR: desired heading is incorrect");
      passing = 0;
      $stop;
    end

    passing = passing && check_west(iPHYS.heading_robot);

    ///////////////////////////
    // Check turning East
    ///////////////////////////

    cmd_east(snd_cmd, cmd, clk);

    fork
      begin : timeout4 
        repeat(10000000) @(negedge clk);
        $error("ERROR: Turning a sequence not performed properly");
        passing = 0;
        $stop;
      end
      // resp_rdy will come later (spi takes time to transmit)
      begin
        @(posedge iDUT.mv_cmplt);
        @(posedge resp_rdy);
        $display("GOOD: turn east complete");
        disable timeout4;
      end
    join

    if(iDUT.dsrd_hdng !== EAST) begin
      $error("ERROR: desired heading is incorrect");
      passing = 0;
      $stop;
    end

    passing = passing && check_east(iPHYS.heading_robot);

    ///////////////////////////
    // Check turning South
    ///////////////////////////

    cmd_south(snd_cmd, cmd, clk);

    fork
      begin : timeout5 
        repeat(10000000) @(negedge clk);
        $error("ERROR: Turning a sequence not performed properly");
        passing = 0;
        $stop;
      end
      // resp_rdy will come later (spi takes time to transmit)
      begin
        @(posedge iDUT.mv_cmplt);
        @(posedge resp_rdy);
        $display("GOOD: turn south complete");
        disable timeout5;
      end
    join

    if(iDUT.dsrd_hdng !== SOUTH) begin
      $error("ERROR: desired heading is incorrect");
      passing = 0;
      $stop;
    end

    passing = passing && check_south(iPHYS.heading_robot);
    
    ///////////////////////////
    // Check turning North
    ///////////////////////////

    cmd_north(snd_cmd, cmd, clk);

    fork
      begin : timeout6 
        repeat(10000000) @(negedge clk);
        $error("ERROR: Turning a sequence not performed properly");
        passing = 0;
        $stop;
      end
      // resp_rdy will come later (spi takes time to transmit)
      begin
        @(posedge iDUT.mv_cmplt);
        @(posedge resp_rdy);
        $display("GOOD: turn north complete");
        disable timeout6;
      end
    join

    if(iDUT.dsrd_hdng !== NORTH) begin
      $error("ERROR: desired heading is incorrect");
      passing = 0;
      $stop;
    end

    passing = passing && check_north(iPHYS.heading_robot);

    //////////////////////////////////////////////////////////////////////////
    // Check we  have not drifted out of the starting square
    //////////////////////////////////////////////////////////////////////////
    passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,3);

    ///////////////////////////////////////////////////////////////////////////////////////
    // Now we have tried turning in all directions, lets try turing in random directions
    // go through 16 random headings. steps:
    // 1. send cmd
    // 2. check timeout
    // 3. check that we are at commanded heading
    // 4. rpt 16 times
    ///////////////////////////////////////////////////////////////////////////////////////

    $display("---STARTING RANDOM TURNING---");

    for (int i = 0; i < 16; i = i + 1) begin
      turn_dir = $urandom_range(0,3);

      case (turn_dir)
        // test turning north
        2'h0 : begin
          cmd_north(snd_cmd, cmd, clk);
          command_timeout(passing_timeout, resp, resp_rdy, clk);
          passing = passing_timeout & passing;
          passing = passing && check_north(iPHYS.heading_robot);
        end
        // test turning east
        2'h1 : begin
          cmd_east(snd_cmd, cmd, clk);
          command_timeout(passing_timeout, resp, resp_rdy, clk);
          passing = passing_timeout & passing;
          passing = passing && check_east(iPHYS.heading_robot);
        end
        // test turning south
        2'h2 : begin
          cmd_south(snd_cmd, cmd, clk);
          command_timeout(passing_timeout, resp, resp_rdy, clk);
          passing = passing_timeout & passing;
          passing = passing && check_south(iPHYS.heading_robot);
        end
        // test turning west
        2'h3 : begin
          cmd_west(snd_cmd, cmd, clk);
          command_timeout(passing_timeout, resp, resp_rdy, clk);
          passing = passing_timeout & passing;
          passing = passing && check_west(iPHYS.heading_robot);
        end
      endcase
    end

    //////////////////////////////////////////////////////////////////////////
    // Check we  have not drifted out of the starting square
    //////////////////////////////////////////////////////////////////////////
    passing = passing & check_square(iPHYS.xx, iPHYS.yy, 3,3);


    verify_passed(passing);
    $stop();
  end

  always
    #5 clk = ~clk;
	
endmodule


