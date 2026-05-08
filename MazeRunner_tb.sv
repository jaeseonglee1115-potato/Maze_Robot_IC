module MazeRunner_tb();
  
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

  ///// Internal registers for testing purposes??? /////////

  
  logic passing;

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
               .cmd_sent(cmd_sent), .resp_rdy(resp_rdy), .resp(resp));
			   
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


     send_cmd = 0;

     batt = 12'hD80; // nominal battery voltage, should not be low enough to trigger low batt behavior
   

    
    clk = 0;
    RST_n = 0;
    // DO NOT send any commands, we want to make sure that nothing happens without being commanded
   
    // wait to deassert reset
    @(posedge clk);
    @(negedge clk);
      //wait 2^17 cycles, which is a pretty long timeu

      RST_n = 1;

        repeat (131072) @(posedge clk); // wait some cycles after reset to let everything initialize


    passing = 1; // assume passing until we find a failure case

    // Check the outputs of the robot, it shouldn't move, send response, fanfair or ANYTHING else
     for (int i = 0; i < 1000000; i++) begin

      @(negedge clk);

      heading_prev  = iPHYS.heading_robot;
      $display("at loop %d", i);
      // ROBOT WILL DRIFT TO SOME EXTENT, DUE TO PWM NOT HAVING EVEN DUTY CYCLE
      // if (!piezo) begin;
      //    $display("ERROR: piezo must not be active");
      //    passing = 0;
      //    $stop;
      //  end


      if (resp_rdy) begin
        $display("ERROR: Robot should not repond when no command was given" );
        passing = 0;
        $stop;
      end
        

      // we can be a little lenient on the PWM values since they might not be exactly midrail, but they should be close
      if(iPHYS.cntrIR != 12'hFFF ) begin
        $display("ERROR: Robot should not move when no command was given  %h", iPHYS.cntrIR );
        passing = 0;
        $stop;
      end


      if(|iPHYS.heading_v > 16'h00FF)  begin
        $display("ERROR: Robot should not change heading when no command was given %d", iPHYS.heading_v);
        passing = 0;
        $stop;
      end
    

      if(iPHYS.xx != 15'h3800 || iPHYS.yy != 15'h3800) begin
        $display("ERROR: Robot should not change position when no command was given %d, %d", iPHYS.xx, iPHYS.yy);
        passing = 0;
        $stop;
      end
      
  
     if(iPHYS.alpha_lft > 13'sd70 || iPHYS.alpha_lft < -13'sd70 || iPHYS.alpha_rght > 13'sd70 || iPHYS.alpha_rght < -13'sd70) begin
        $display("ERROR: Robot should not change wheel angles when no command was given  %d, %d", iPHYS.alpha_lft, iPHYS.alpha_rght);
      passing = 0;
        $stop();
      end

      if(iPHYS.omega_lft < -16'sd70 || iPHYS.omega_lft > 16'sd70 || iPHYS.omega_rght < -16'sd70 || iPHYS.omega_rght > 16'sd70) begin
        $display("ERROR: Robot should not change wheel angular velocities when no command was given %d, %d", iPHYS.omega_lft, iPHYS.omega_rght);
        passing = 0;
        $stop();

      end
    



      if (iPHYS.mazeModel[0][0] != 4'h5 || iPHYS.mazeModel[1][0] != 4'h6 ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[0][0], iPHYS.mazeModel[1][0]);
        passing = 0;
        $stop();
      end


    if(iPHYS.mazeModel[2][0] != 4'h3 || iPHYS.mazeModel[3][0] != 4'h7 ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[2][0], iPHYS.mazeModel[3][0]);
        passing = 0;
        $stop();
      end


    if (iPHYS.mazeModel[0][1] != 4'h3 || iPHYS.mazeModel[1][1] != 4'h1 ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[0][1], iPHYS.mazeModel[1][1]);
        passing = 0;
        $stop();
      end

    if (iPHYS.mazeModel[2][1] != 4'hA || iPHYS.mazeModel[3][1] != 4'h3 ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[2][1], iPHYS.mazeModel[3][1]);
        passing = 0;
        $stop();
      end


    if (iPHYS.mazeModel[0][2] != 4'h3 || iPHYS.mazeModel[1][2] != 4'h9 ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[0][2], iPHYS.mazeModel[1][2]);
        passing = 0;
        $stop();
      end

    if(iPHYS.mazeModel[2][2] != 4'hC || iPHYS.mazeModel[3][2] != 4'h2 ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[2][2], iPHYS.mazeModel[3][2]);
        passing = 0;
        $stop();
      end

    if (iPHYS.mazeModel[0][3] != 4'h9 || iPHYS.mazeModel[1][3] != 4'hC ) begin

        $display("ERROR: Maze model should be initialized to correct values %d,%d" , iPHYS.mazeModel[0][3], iPHYS.mazeModel[1][3]);
        passing = 0;
        $stop();
      end


    if(~iPHYS.iNEMO.NEMO_setup) begin
        passing = 0;
    end
    else begin
      passing = 1;
    end

      
    end

    if (passing) begin
      $display("PASS: Robot power-up behavior is correct");
    end else begin
      $display("FAIL: Robot power-up behavior is incorrect");
    end
    $stop();
  end

 
	

  // This task sends a command to the robot
  task send_command(input [15:0] cmd, output snd_cmd, output out_port);
    assign snd_cmd = 1'b0;
    @(negedge clk);
    assign out_port = cmd;
    assign snd_cmd = 1'b1;
    @(negedge clk);
    assign snd_cmd = 1'b0;
  endtask

  // Tell robot to calibrate
  task cmd_calibrate(output snd_cmd, output out_port);
      send_command({3'b000, 1'b0, 12'h000}, snd_cmd, out_port);
  endtask

  // Tell robot to go north
  task cmd_north(output snd_cmd, output out_port);
      send_command({3'b001, 1'b0, 12'h000}, snd_cmd, out_port);
  endtask

  // Tell robot to go east
  task cmd_east(output snd_cmd, output out_port);
      send_command({3'b001, 1'b0, 12'hC00}, snd_cmd, out_port);
  endtask

  // Tell robot to go south
  task cmd_south(output snd_cmd, output out_port);
      send_command({3'b001, 1'b0, 12'h7FF}, snd_cmd, out_port);
  endtask

  // Tell robot to go west
  task cmd_west(output snd_cmd, output out_port);
      send_command({3'b001, 1'b0, 12'h3FF}, snd_cmd, out_port);
  endtask

  // Tell robot to move with left affinity
  task cmd_move_lft(output snd_cmd, output out_port);
      send_command({3'b010, 1'b0, 12'h002}, snd_cmd, out_port);
  endtask

  // Tell robot to move with right affinity
  task cmd_move_rght(output snd_cmd, output out_port);
      send_command({3'b010, 1'b0, 12'h001}, snd_cmd, out_port);
  endtask

  // Tell robot to solve with left affinity
  task cmd_solve_lft(output snd_cmd, output out_port);
      send_command({3'b010, 1'b0, 12'h001}, snd_cmd, out_port);
  endtask

  // Tell robot to solve with right affinity
  task cmd_solve_rgft(output snd_cmd, output out_port);
      send_command({3'b010, 1'b0, 12'h000}, snd_cmd, out_port);
  endtask

  always
    #5 clk = ~clk;
	
endmodule
