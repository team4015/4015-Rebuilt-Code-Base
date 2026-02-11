// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.RobotContainer;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
    private AnalogInput frontLeft = new AnalogInput(DRIVE.encoders.absolutePorts.frontLeft);
    private AnalogInput frontRight = new AnalogInput(DRIVE.encoders.absolutePorts.frontRight);
    private AnalogInput backLeft = new AnalogInput(DRIVE.encoders.absolutePorts.backLeft);
    private AnalogInput backRight = new AnalogInput(DRIVE.encoders.absolutePorts.backRight);
    private ShuffleboardTab tab;

    @Override
    public void robotInit() {
      robotContainer = new RobotContainer();
      tab = Shuffleboard.getTab("Encoder Offsets");

      tab.addNumber("Front Left Absolute Encoder", () -> getAbsoluteEncoderRad(frontLeft));
      tab.addNumber("Front Right Absolute Encoder", () -> getAbsoluteEncoderRad(frontRight));
      tab.addNumber("Back Left Absolute Encoder", () -> getAbsoluteEncoderRad(backLeft));
      tab.addNumber("Back Right Absolute Encoder", () -> getAbsoluteEncoderRad(backRight));
    }

    public double getAbsoluteEncoderRad(AnalogInput absoluteEncoder){
        //gets the voltages from the encoder then get the actual rail voltage which is divided to get the angle
        double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
        angle *= 2.0 * Math.PI; //convert this angle into radians
        return angle; //result the angles and multiples by 1 or -1 if the absolute encoder is reversed
    }


    @Override
    public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
    }

    @Override
    public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}