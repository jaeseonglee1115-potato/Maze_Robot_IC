package test_suite;
    
  localparam speed_thresholds_high  = 13'sd100;  // upper and lower thresholds for angular and regular wheell velocity readings
  localparam speed_thresholds_low  = -13'sd100;  // to prevent the drifitnhg from causing tb error s
  localparam default_reset_pos =  15'h3800;   // for xx and yy positions, this is what its reset to 

  localparam WEST = 12'h3FF;
  localparam EAST = 12'hc00;
  localparam NORTH = 12'h000;
  localparam SOUTH = 12'h7FF;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // checking angular and forward velocities, accounting for some drift, in iether direction 
  ///////////////////////////////////////////////////////////////////////////////////////////
  task  automatic check_moving(input signed [15:0] r_alpha_lft, input signed [15:0] r_alpha_rght,
                                input signed [15:0] r_omega_lft, input signed [15:0] r_omega_rght, 
                                 ref logic passing); 

    // if wheet acceleration positive, then the robot is moving
    if(r_alpha_lft > speed_thresholds_high || r_alpha_lft < speed_thresholds_low || 
        r_alpha_rght > speed_thresholds_high || r_alpha_rght < speed_thresholds_low) begin
      $display("ERROR: Robot should not change wheel angles when no command was given  %d, %d", r_alpha_lft,r_alpha_rght);
    passing = 0;
      $stop();
    end

    // if wheet speed or speed is positive, then the robot is moving
    if(r_omega_lft < speed_thresholds_low|| r_omega_lft > speed_thresholds_high|| 
        r_omega_rght < speed_thresholds_low|| r_omega_rght > speed_thresholds_high) begin
      $display("ERROR: Robot should not change wheel angular velocities when no command was given %d, %d", r_omega_lft, r_omega_rght);
      passing = 0;
      $stop();
    end
      
  endtask

  ///////////////////////////////////////////////////////////////////////////////////////////
  // designed to test timeout when command is sent.
  // waits for 10000000 clock cycles before timeout error.
  // checks that the received resp is correct
  ///////////////////////////////////////////////////////////////////////////////////////////
  task automatic command_timeout(output passing, ref [7:0]resp, ref resp_rdy, ref clk);

    // wait for response or timeout
    fork
      begin : timeout 
        repeat(10000000) @(negedge clk);
        $error("ERROR: timeout");
        passing = 0;
        $stop;
      end
      begin
        @(posedge resp_rdy);
        $display("GOOD: move completeted");
        passing = 1;
        disable timeout;
      end
    join
  
    // check the response
    if(resp === 8'hA5) begin
      $display("GOOD: reponse is correct");
    end else begin
      $error("ERROR: response incorrect, EXP: 8'hA5 REC: %h", resp);
    end

  endtask

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Tasks for telling remote_comm to communicate with the robot and send command.
  // automatically applies stimulus
  ///////////////////////////////////////////////////////////////////////////////////////////

  // This task sends a command to the robot
  task  automatic send_command(input [15:0] cmd ,ref logic snd_cmd, ref logic [15:0] out_port, ref logic clk);
     snd_cmd = 1'b0;
    @(negedge clk);
     out_port = cmd;
     snd_cmd = 1'b1;
    @(negedge clk);
     snd_cmd = 1'b0;
  endtask

  // Tell robot to calibrate
  task  automatic cmd_calibrate(ref logic snd_cmd, ref logic [15:0] out_port, ref logic clk);
      send_command({3'b000, 1'b0, 12'h000}, snd_cmd, out_port,clk);
  endtask

  // Tell robot to go north
  task  automatic cmd_north(ref logic snd_cmd, ref logic [15:0] out_port,  ref logic clk);
      send_command({3'b001, 1'b0, 12'h000}, snd_cmd, out_port,  clk);
  endtask

  // Tell robot to go east
  task  automatic cmd_east(ref logic snd_cmd, ref logic [15:0] out_port, ref logic clk);
      send_command({3'b001, 1'b0, 12'hC00}, snd_cmd, out_port, clk);
  endtask

  // Tell robot to go south
  task  automatic cmd_south(ref logic snd_cmd, ref logic [15:0] out_port, ref logic clk);
      send_command({3'b001, 1'b0, 12'h7FF}, snd_cmd, out_port,  clk);
  endtask

  // Tell robot to go west
  task  automatic cmd_west(ref logic snd_cmd, ref logic [15:0] out_port,  ref logic clk);
      send_command({3'b001, 1'b0, 12'h3FF}, snd_cmd, out_port,  clk);
  endtask

  // Tell robot to move with left affinity
  task  automatic cmd_move_lft(ref logic snd_cmd, ref logic [15:0] out_port,  ref logic clk);
      send_command({3'b010, 1'b0, 12'h002}, snd_cmd, out_port,  clk);
  endtask

  // Tell robot to move with right affinity
  task  automatic cmd_move_rght(ref logic snd_cmd, ref logic [15:0] out_port, ref logic clk);
      send_command({3'b010, 1'b0, 12'h001}, snd_cmd, out_port,  clk);
  endtask

  // Tell robot to solve with left affinity
  task  automatic cmd_solve_lft(ref logic snd_cmd, ref logic [15:0] out_port,  ref logic clk);
      send_command({3'b011, 1'b0, 12'h001}, snd_cmd, out_port, clk);
  endtask

  // Tell robot to solve with right affinity
  task  automatic cmd_solve_rght(ref logic snd_cmd, ref logic [15:0] out_port, ref logic clk);
      send_command({3'b011, 1'b0, 12'h000}, snd_cmd, out_port,clk);
  endtask

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Evaluate test results based on passing - called at end of test
  ///////////////////////////////////////////////////////////////////////////////////////////

  task automatic verify_passed(ref passing);
    if (passing) begin
      $display("PASS: Robot passed test");
    end
    else begin
      $display("FAIL: Robot passed test");
    end
    $stop();
  endtask

  ///////////////////////////////////////////////////////////////////////////////////////////
  // functions for checking the direction expect within ~|20deg| - gives space for gyro error
  // (tolerance/(2^n-1)~=20/360)
  ///////////////////////////////////////////////////////////////////////////////////////////

  function bit check_north(input [19:0]heading);
    // need to handle differently due to wrap-around of heading at 0xfff and 0x000
    if ((heading[19:8] < -12'sh0f0 && heading[19:8] > 12'sh0f0) || 
        (heading[19:8] > 12'sh0f0 && heading[19:8] < -12'sh0f0)) begin 
      check_north = 0;
      $error("ERROR: robot is more than 5 degrees off north");
      $stop;
    end else begin
      check_north = 1;
      $display("GOOD: robot is pointing north");
    end

  endfunction

  function bit check_east(input [19:0]heading);

    if ((heading[19:8] < EAST - 12'sh0f0 || heading[19:8] > EAST + 12'sh0f0)) begin 
      check_east = 0;
      $error("ERROR: robot is more than 5 degrees off east");
      $stop;
    end else begin
      check_east = 1;
      $display("GOOD: robot is pointing east");
    end

  endfunction

  function bit check_south(input [19:0]heading);

    if ((heading[19:8] < SOUTH - 12'sh0f0 || heading[19:8] > SOUTH + 12'sh0f0)) begin 
      check_south = 0;
      $error("ERROR: robot is more than 5 degrees off south");
      $stop;
    end else begin
      check_south = 1;
      $display("GOOD: robot is pointing south");
    end
  endfunction

  function bit check_west(input [19:0]heading);

    if ((heading[19:8] < WEST - 12'sh0f0 || heading[19:8] > WEST + 12'sh0f0)) begin 
      check_west = 0;
      $error("ERROR: robot is more than 5 degrees off west");
      $stop;
    end else begin
      check_west = 1;
      $display("GOOD: robot is pointing west");
    end
  endfunction

/////////////////////////////////////////////////////////////////////////////
// Tests if robot coordinates (xx, yy) are in the specified square
// as inidcated by bits 13;12 of xx and yy.
/////////////////////////////////////////////////////////////////////////////
function bit check_square(input [14:0]xx, input [14:0]yy, 
            input [1:0]x_square, input [1:0]y_square);
  // [13:12] correspond to coordinated of cell in maze_model
  if(inside_square(xx, yy, x_square, y_square)) begin
    $display("GOOD: robot is in expected square: %h, %h", x_square, y_square);
    check_square = 1;
  end else begin
    $error("ERROR: not in expected square: %h, %h", x_square, y_square);
    $stop;
    check_square = 0;
  end
endfunction

/////////////////////////////////////////////////////////////////////////////
// Checks that given corrdinates (xx, yy) are in the specified square (x, y)
// bits 13:12 of  xx and yy indicate square
/////////////////////////////////////////////////////////////////////////////
function bit inside_square(input [14:0]xx, input [14:0]yy, 
            input [1:0]x_square, input [1:0]y_square);
  // [13:12] correspond to coordinated of cell in maze_model
  inside_square = (x_square === xx[13:12]) & (y_square === yy[13:12]);
endfunction

endpackage