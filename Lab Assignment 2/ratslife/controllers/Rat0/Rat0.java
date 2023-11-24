// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.LightSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import java.util.Random;

public class Rat0 extends Robot {

  protected final int timeStep = 32;
  protected final double maxSpeed = 300;
  protected final double[] collisionAvoidanceWeights = {0.06,0.03,0.015,0.0,0.0,-0.015,-0.03,-0.06};
  protected final double[] slowMotionWeights = {0.0125,0.00625,0.0,0.0,0.0,0.0,0.00625,0.0125};

  protected Accelerometer accelerometer;
  protected Camera camera;
  protected int cameraWidth, cameraHeight;
  protected Motor leftMotor, rightMotor;
  protected DistanceSensor[] distanceSensors = new DistanceSensor[8];
  protected LightSensor[] lightSensors = new LightSensor[8];
  protected LED[] leds = new LED[10];

  public Rat0() {
    accelerometer = getAccelerometer("accelerometer");
    camera = getCamera("camera");
    camera.enable(8*timeStep);
    cameraWidth=camera.getWidth();
    cameraHeight=camera.getHeight();
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    for (int i=0;i<10;i++) {
      leds[i]=getLED("led"+i);
    };
    for (int i=0;i<8;i++) {
      distanceSensors[i] = getDistanceSensor("ps"+i);
      distanceSensors[i].enable(timeStep);
      lightSensors[i] = getLightSensor("ls"+i);
      lightSensors[i].enable(timeStep);
    }
    batterySensorEnable(timeStep);
  }

  public void run() {

    int blink = 0;
    int oldDx = 0;
    Random r = new Random();
    boolean turn = false;
    boolean right = false;
    boolean seeFeeder = false;
    double battery;
    double oldBattery = -1.0;
    int image[];
    double distance[] = new double[8];
    int ledValue[] = new int[10];
    double leftSpeed, rightSpeed;

    while (step(timeStep) != -1) {

      // read sensor information
      image = camera.getImage();
      for(int i=0;i<8;i++) distance[i] = distanceSensors[i].getValue();
      battery = batterySensorGetValue();
      
      //Initially set the two motors at max speed
      leftSpeed = maxSpeed;
      rightSpeed = maxSpeed;
      
      //Obstacle detected in front 
      if(distance[4] > 400 || distance[3] > 400){
        //Turn left (dir of movement) to avoid it and to obey to the L.W.F. rule
        leftSpeed = -maxSpeed;
        rightSpeed = maxSpeed;
      }
      else{ 
        //The distance from the left hand side wall is okay, i obey to the LWF rule continue straight
        if(distance[2] > 310){
          //System.out.println("Moving on...");
          leftSpeed = maxSpeed;
          rightSpeed = maxSpeed;
        }
        else{
          //In case we lost contact with left wall
          leftSpeed = maxSpeed;
          rightSpeed = maxSpeed/8;
        }
        
        //I had an 180 left turn, avoid crash on wall
        if(distance[3] > 200){
          leftSpeed = maxSpeed/2.5;
          rightSpeed = maxSpeed;
        }
      }
      
      int blobX=0,blobY=0,blobCounter=0;
      // looking for an alight feeder
      for(int x=0; x<cameraWidth; x++) for(int y=cameraWidth/3; y<2*cameraWidth/3; y++) {
        int pixel = image[y * cameraWidth + x];
        if (Camera.pixelGetGreen(pixel) >= 248 &&
            Camera.pixelGetBlue(pixel) >= 248) {
          blobX += x;
          blobY += y;
          blobCounter++;
        }
      }
      if (blobCounter > 2) { // significant enough
        seeFeeder = true;
        blobX /= blobCounter;
        blobY /= blobCounter;
        int dx = (blobX-cameraWidth/2)*10;
        if (dx > 0) ledValue[1] = 1;
        else ledValue[7] = 1;
        if (oldDx != dx){
          leftSpeed  =  2*dx;
          rightSpeed = -2*dx;
          oldDx = dx;
        } else {
          leftSpeed  = maxSpeed/3;
          rightSpeed = maxSpeed/3;
        }
      }
    
      //recharging behavior
      if (battery > oldBattery) {
        leftSpeed  = 0.0;
        rightSpeed = 0.0;
      }
      oldBattery = battery;
      if (blink++ >= 20) { // blink the back LEDs
        if (blink == 40) blink = 0;
      }
      
      //Reverse 
      leftMotor.setVelocity(-0.00628 * leftSpeed);
      rightMotor.setVelocity(-0.00628 * rightSpeed);
      // Enter here exit cleanup code
    }
  }

  public static void main(String[] args) {
    Rat0 rat0 = new Rat0();
    rat0.run();
  }
}
