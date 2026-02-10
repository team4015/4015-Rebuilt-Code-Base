// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
        new SwerveJoystickCommand(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
        ));

    configureBindings();
  }

  private void configureBindings() {
    // Bind buttons/triggers here when needed.
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
