//-----------------------------------------------------------------------------
//  File:         FieldPlayer.java (to be used in a Webots java controllers)
//  Date:         April 30, 2008
//  Description:  Field player "2", "3" or "4" for "red" or "blue" team
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Changes:      November 4, 2008: Adapted to Webots6
//-----------------------------------------------------------------------------

import com.cyberbotics.webots.controller.*;

public class FieldPlayer extends Player {

  private Motion backwardsMotion, forwardsMotion, forwards50Motion, turnRight40Motion, turnLeft40Motion;
  private Motion turnRight60Motion, turnLeft60Motion, turnLeft180Motion, sideStepRightMotion, sideStepLeftMotion;

  // Added shooting motion
  private Motion shootingMotion;

  private double goalDir = 0.0; // interpolated goal direction (with respect to front direction of robot body)

  public FieldPlayer(int playerID, int teamID) {
    super(playerID, teamID);
    backwardsMotion     = new Motion("../../motions/Backwards.motion");
    forwardsMotion      = new Motion("../../motions/Forwards.motion");
    forwards50Motion    = new Motion("../../motions/Forwards50.motion");
    turnRight40Motion   = new Motion("../../motions/TurnRight40.motion");
    turnLeft40Motion    = new Motion("../../motions/TurnLeft40.motion");
    turnRight60Motion   = new Motion("../../motions/TurnRight60.motion");
    turnLeft60Motion    = new Motion("../../motions/TurnLeft60.motion");
    turnLeft180Motion   = new Motion("../../motions/TurnLeft180.motion");
    sideStepRightMotion = new Motion("../../motions/SideStepRight.motion");
    sideStepLeftMotion  = new Motion("../../motions/SideStepLeft.motion");
    
    // init motion
    shootingMotion = new Motion("../../motions/Shoot.motion");

    // move arms along the body
    Motor leftShoulderPitch = getMotor("LShoulderPitch");
    Motor rightShoulderPitch = getMotor("RShoulderPitch");
    leftShoulderPitch.setPosition(1.5);
    rightShoulderPitch.setPosition(1.5);
  }

  // normalize angle between -PI and +PI
  private double normalizeAngle(double angle) {
    while (angle > Math.PI) angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
  }

  // relative body turn
  private void turnBodyRel(double angle) {
    if (angle > 0.7)
      turnRight60();
    else if (angle < -0.7)
      turnLeft60();
    else if (angle > 0.3)
      turnRight40();
    else if (angle < -0.3)
      turnLeft40();
  }

  protected void runStep() {
    super.runStep();
    double dir = camera.getGoalDirectionAngle();
    if (dir != NaoCam.UNKNOWN)
      goalDir = dir - headYawPosition.getValue();
  }

  private void turnRight60() {
    playMotion(turnRight60Motion); // 59.2 degrees
    goalDir = normalizeAngle(goalDir - 1.033);
  }

  private void turnLeft60() {
    playMotion(turnLeft60Motion); // 59.2 degrees
    goalDir = normalizeAngle(goalDir + 1.033);
  }

  private void turnRight40() {
    playMotion(turnRight40Motion); // 39.7 degrees
    goalDir = normalizeAngle(goalDir - 0.693);
  }

  private void turnLeft40() {
    playMotion(turnLeft40Motion); // 39.7 degrees
    goalDir = normalizeAngle(goalDir + 0.693);
  }
  
  private void turnLeft180() {
    playMotion(turnLeft180Motion); // 163.6 degrees
    goalDir = normalizeAngle(goalDir + 2.855);
  }


  @Override public void run(){
    
    boolean kickoff = true;
    
    step(SIMULATION_STEP);

    while(true){
      
      runStep();

      getUpIfNecessary();

      while(getBallDirection() == NaoCam.UNKNOWN) {
        
        getUpIfNecessary();

        System.out.println("Searching the Ball...");

        if(getBallDirection() != NaoCam.UNKNOWN) 
          break;

        headScan();

        if(getBallDirection() != NaoCam.UNKNOWN) 
          break;
        
        playMotion(backwardsMotion);

        if(getBallDirection() != NaoCam.UNKNOWN) 
          break;

        headScan();
        
        if(getBallDirection() != NaoCam.UNKNOWN) 
          break;

        turnLeft180();

      }

      double ballDir = getBallDirection();
      double ballDist = getBallDistance();

      System.out.println("ball dist: " + ballDist + " ball dir: " + ballDir + " goal dir: " + goalDir);

      if(ballDist < 0.5) {

        // goal dir is initialized at early state to 0.0, each turn assigns a new (float) value to the target
        // we have to make changes in order to shoot from different angles.

        // in order to shoot and to face goal we have to set a range on goalDir value
        // Simulating many scenarios gave as the opportunity to see that, if the goalDir
        // is between [-0.5, 0.5] then the robot a great shoot direction

        // if kickoff the make a specific movement
        if(kickoff) {
          if (ballDist > 0.25){
            playMotion(forwardsMotion);
          }
          else{
            // step(SIMULATION_STEP);
           // sleepSteps(1);
            playMotion(forwardsMotion);
            playMotion(sideStepRightMotion);          
            playMotion(shootingMotion);
            kickoff = false;
          }
        } 
        else {

          while(ballDist > 0.25){
            playMotion(forwardsMotion);
            ballDist = getBallDistance();
            ballDir = getBallDirection();
            step(SIMULATION_STEP);
          }

          if (ballDist < 0.10){
            playMotion(backwardsMotion);
          } 
          else{
            if(goalDir == 0.0){
              // robot-ball allignment is perfect
              if(ballDir > 0 && ballDir < 0.12){
                step(SIMULATION_STEP);
                playMotion(forwardsMotion);
                playMotion(sideStepRightMotion);
                playMotion(shootingMotion);
              }
              else if( ballDir < 0 && ballDir > -0.12){
                
                // do kickoff maneuver
                if(ballDir<0 && ballDir > -0.1){
                  step(SIMULATION_STEP);
                  playMotion(forwardsMotion);
                  playMotion(sideStepRightMotion);   
                  playMotion(shootingMotion);
                }else{
                  step(SIMULATION_STEP);
                  playMotion(forwardsMotion);
                  playMotion(sideStepLeftMotion);          
                  playMotion(shootingMotion);
                }
                
              }
              // robot-ball allignment is not perfect
              else if(ballDir > 0.13){ // -> left relative to the ball
                System.out.println("I need to go right");
                while(ballDir > 0.13){
                  getUpIfNecessary();
                  if(ballDir > 0 && ballDir < 0.12){
                    step(SIMULATION_STEP);
                    playMotion(forwardsMotion);
                    playMotion(sideStepRightMotion);
                    playMotion(shootingMotion);
                    break;
                  }
                  else
                  {
                    playMotion(sideStepRightMotion);
                    ballDir = getBallDirection();
                    step(SIMULATION_STEP);
                  }                  
                  
                }
              }
              else if (ballDir < -0.13){           
                System.out.println("I need to go left");
                while(ballDir < -0.13){// -> right relative to the ball
                  getUpIfNecessary();

                  if (ballDir < -0.13 && ballDir > -0.16){
                    step(SIMULATION_STEP);
                    playMotion(forwardsMotion);
                    playMotion(sideStepRightMotion);
                    playMotion(shootingMotion);
                    break;
                  }
                  else 
                  {
                    playMotion(sideStepLeftMotion);
                    ballDir = getBallDirection();
                    step(SIMULATION_STEP);
                  }
                  
                }             
              }

            }
            else{
              // if goaldir != 0 then we have to align the goal

              // by turning left we add in goaldir
              // by turing right we subtract in goaldir
                  getUpIfNecessary();

              while (Math.abs(goalDir) > 0.05) {
                  if (goalDir > 0.05) {
                      turnRight40(); // adjust the angle as needed
                  } else if (goalDir < -0.05) {
                      turnLeft40(); // adjust the angle as needed
                  }

                  goalDir = normalizeAngle(goalDir);
              }
              
            }

          }

        }

      }
      else{
        //System.out.println("long distance");
        double goDir = normalizeAngle(goalDir - ballDir);

        if (goDir < ballDir - 0.5)
          goDir = ballDir - 0.5;
        else if (goDir > ballDir + 0.5)
          goDir = ballDir + 0.5;

        goDir = normalizeAngle(goDir);

        turnBodyRel(goDir);
        if (ballDist < 0.6)
          playMotion(forwardsMotion);
        else
          playMotion(forwards50Motion);
      }
    }
  }

}
