package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(Constants.OIConstants.driverControllerPort);

  public RobotContainer(){
    swerveSubsystem.setDefaultCommand(
      new SwerveCommands(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.driverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.driverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.driverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.driverFieldOrientedButtonIdx)
        )
      );

      configureBindings();
  }

  private void configureBindings(){
      new JoystickButton(driverJoystick, OIConstants.driveUnlockSwerveButtonIdx)
              .whileTrue(
                      Commands.startEnd(
                              () -> {
                                  swerveSubsystem.setBrakeMode(false);
                                  swerveSubsystem.stopModules();
                              },
                              () -> {
                                  swerveSubsystem.setBrakeMode(true);
                                  swerveSubsystem.stopModules();
                              },
                              swerveSubsystem
                      )
              );
  }

  public SwerveSubsystem getSwerveSubsystem(){
      return swerveSubsystem;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
