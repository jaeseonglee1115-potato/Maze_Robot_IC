NEED to decide to decde which modules we're using
this involve sub modules too

cmd proc -  ishaan and janice
motor drive - jae and noah
mazeSolve - ishaan and janice
piezo - noah and jae
inert_intf  - noah

remote commm - undecided
navigate - ishaan and janice
uart-  ishaan  


TESTING PROCEIDURE INFORMATION:
 1. POWER ON: 
	- should not do ANYTHING unless commanded
	- check speed, heading, PIEZO, etc... are OFF
 2. CALIBRATE:
	- should not do anything, finished when cal_done asserted
	- check speed, heading, PIEZO, etc... are OFF
 3. CHANGE HEADING:
	- Check that PWMs are correct speed.
	- check heading (are we overshooting? Something else?)
	- make sure at_hdng asserted when at heading, and that it stays true
 4. MOVE COMMAND:
	- Check that we aren't moving into walls
	- Check ....
 5. ...


We need 9 tasks for the tests:
calibration
moving
heading  ->  4 different tasks because we have  4 different directions
task_solve
send_cmd() 
check_heading  -> desired hading is correct?



 for example:
  go_west() ---> sends a command op code that indicates giong west and does so via send command










