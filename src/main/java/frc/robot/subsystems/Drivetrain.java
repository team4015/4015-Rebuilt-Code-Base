// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Define the motors. Based on the type of motor controller, you'd use the corresponding class to define the motor controllers in code.
  // For example, if you're using VictorSP motor controllers, you'd use that class.
  private VictorSP leftFront = new VictorSP(0);
  private VictorSP leftRear = new VictorSP(1);
  
  private VictorSP rightFront = new VictorSP(2);
  private VictorSP rightRear = new VictorSP(3);


  // Define the slew rate limiters, which smooth out the acceleration so the robot doesn't accelerate like crazy if you tilt the joystick quick.
  // We need to define two of them. One for the 2 left motors, and one for the two right motors.
  private final SlewRateLimiter leftLimiter;
  private final SlewRateLimiter rightLimiter;

  private DifferentialDrive drivetrain;

  //The Drivetrain constructor, which runs when an object of this class is made. 
  //Basically, the method initializing an object that exists in many classes.
  public Drivetrain() {

    //Since the DifferentialDrive constructor only takes two motor controllers, we use the .addFollower(motorController) method to incorporate all 4 motor controllers.
    //Basically, it clones the output of one motor controller to another. For example, if leftFront is outputting a speed of 2, leftRear will also output a speed of 2.
    leftFront.addFollower(leftRear);
    rightFront.addFollower(rightRear);

    //Since the orientation of the motors on the right are flipped compared to the ones on the left, we need to invert the motor controllers on the right.
    
    rightFront.setInverted(true);
    rightRear.setInverted(true);

    //Initializing the drivetrain object of type DifferentialDrive
    
    drivetrain = new DifferentialDrive(leftFront, rightFront);

    //Assigning the slew rate limiters a value. 
    
    leftLimiter = new SlewRateLimiter(3);//The rate of change in speed is proportional to the value of the limiter. 
    rightLimiter = new SlewRateLimiter(3);//In other words, the higher the limiter, the higher the rate of change.


  }

  //This is the method that'll actually be used to move the robot. 
  //The differential drive works by varying the speed of the motors on both the left and right, separately.
  /*This is reflected in the code by passing two separate parameters for the speed of the left and right motors,
  as well as having two slew rate limiters and two deadband method calls. */

  public void drive(double leftSpeed, double rightSpeed){

    //Applying a deadband to the speed. If it's less than the deadband, the robot will ignore the speed value.
    //This is so the robot ignores small and negligible joystick inputs.
    leftSpeed = applyDeadband(leftSpeed, 0.05);
    rightSpeed = applyDeadband(rightSpeed, 0.05);

    //Apply the slew rate limiters to the speed.
    leftSpeed = leftLimiter.calculate(leftSpeed);
    rightSpeed = rightLimiter.calculate(rightSpeed);

    //Drive the robot.
    drivetrain.tankDrive(leftSpeed, rightSpeed, true);

  }

  //This is the method that actually applies a deadband to a value, called in the drive method. 
  public double applyDeadband(double value, double deadband){
    
    if (Math.abs(value) < deadband){//If the absolute value of the speed value (given by the joystick) is less than the deadband,
      return 0; //set the speed to 0.
    } 
    return value; // Otherwise, keep the speed as it is.
  }

  //Stop driving the robot. Pretty self-explanatory.
  public void stop(){
    drivetrain.tankDrive(0, 0);
  }

}
