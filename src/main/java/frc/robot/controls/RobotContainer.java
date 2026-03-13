package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

/**
 * Builds the robot's subsystems, commands, and controller bindings.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(swerveSubsystem, limelightSubsystem);
    private final Joystick driverJoystick = new Joystick(Constants.OIConstants.driverControllerPort);

    /**
     * Creates the robot container and installs default commands and button bindings.
     */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(
            new SwerveCommands(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.driverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverRotAxis),
                // Default to field-oriented driving; hold the button to switch to robot-centric.
                () -> !driverJoystick.getRawButton(OIConstants.driverFieldOrientedButtonIdx),
                () -> driverJoystick.getRawButton(OIConstants.aimAtTagButtonIdx),
                limelightSubsystem,
                shooterSubsystem
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, OIConstants.shooterToggleButtonIdx)
            .onTrue(new ShooterCommand(shooterSubsystem));

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

    /**
     * Returns the swerve subsystem.
     *
     * @return swerve subsystem
     */
    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    /**
     * Returns the Limelight subsystem.
     *
     * @return vision subsystem
     */
    public LimelightSubsystem getLimelightSubsystem() {
        return limelightSubsystem;
    }

    /**
     * Returns the shooter subsystem.
     *
     * @return shooter subsystem
     */
    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    /**
     * Returns the autonomous command to schedule when autonomous begins.
     *
     * @return autonomous command, or {@code null} if none is configured
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
