// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
      
  private VictorSP leftFront = new VictorSP(0);
  private VictorSP leftRear = new VictorSP(1);
  
  private VictorSP rightFront = new VictorSP(2);
  private VictorSP rightRear = new VictorSP(3);

  private final SlewRateLimiter leftLimiter;
  private final SlewRateLimiter rightLimiter;

  private final DigitalInput m_limitSwitch = new DigitalInput(0);

  private DifferentialDrive drivetrain;
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    leftFront.addFollower(leftRear);
    rightFront.addFollower(rightRear);

    rightFront.setInverted(true);
    rightRear.setInverted(true);
    
    drivetrain = new DifferentialDrive(leftFront, rightFront);

    leftLimiter = new SlewRateLimiter(3);
    rightLimiter = new SlewRateLimiter(3);

  }

  public void drive(double leftSpeed, double rightSpeed){
    leftSpeed = applyDeadband(leftSpeed, 0.05);
    rightSpeed = applyDeadband(rightSpeed, 0.05);

    leftSpeed = leftLimiter.calculate(leftSpeed);
    rightSpeed = rightLimiter.calculate(rightSpeed);

    drivetrain.tankDrive(leftSpeed, rightSpeed, true);

  }

  public double applyDeadband(double value, double deadband){
    if (Math.abs(value) < deadband){
      return 0;
    } 
    return value;
  }

  public void stop(){
    drivetrain.tankDrive(0, 0);
  }

}
