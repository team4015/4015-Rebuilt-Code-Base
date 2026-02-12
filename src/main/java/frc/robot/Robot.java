// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private ShuffleboardTab tab;

    @Override
    public void robotInit() {
      robotContainer = new RobotContainer();
      tab = Shuffleboard.getTab("Encoder Offsets");

      SwerveSubsystem swerveSubsystem = robotContainer.getSwerveSubsystem();
      tab.addNumber("Front Left Absolute Encoder", swerveSubsystem::getFrontLeftAbsoluteEncoderRad);
      tab.addNumber("Front Right Absolute Encoder", swerveSubsystem::getFrontRightAbsoluteEncoderRad);
      tab.addNumber("Back Left Absolute Encoder", swerveSubsystem::getBackLeftAbsoluteEncoderRad);
      tab.addNumber("Back Right Absolute Encoder", swerveSubsystem::getBackRightAbsoluteEncoderRad);
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