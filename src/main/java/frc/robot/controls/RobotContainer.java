package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExtendableHopperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Hopper.ExtendableHopperSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

/**
 * Builds the robot's subsystems, commands, and controller bindings.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ExtendableHopperSubsystem extendableHopperSubsystem = new ExtendableHopperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
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
                () -> driverJoystick.getRawButton(OIConstants.driverFieldOrientedButtonIdx),
                () -> driverJoystick.getRawButton(OIConstants.aimAtTagButtonIdx),
                limelightSubsystem,
                shooterSubsystem
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, OIConstants.intakeToggleButtonIdx)
            .onTrue(new IntakeCommand(intakeSubsystem));

        new JoystickButton(driverJoystick, OIConstants.shooterToggleButtonIdx)
            .onTrue(new ShooterCommand(shooterSubsystem));

        new Trigger(() -> driverJoystick.getRawAxis(OIConstants.driverRightTriggerAxis) > OIConstants.triggerPressedThreshold)
            .whileTrue(new ExtendableHopperCommand(extendableHopperSubsystem, false));

        new Trigger(() -> driverJoystick.getRawAxis(OIConstants.driverLeftTriggerAxis) > OIConstants.triggerPressedThreshold)
            .whileTrue(new ExtendableHopperCommand(extendableHopperSubsystem, true));

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
     * Returns the extendable hopper subsystem.
     *
     * @return hopper subsystem
     */
    public ExtendableHopperSubsystem getExtendableHopperSubsystem() {
        return extendableHopperSubsystem;
    }

    /**
     * Returns the intake subsystem.
     *
     * @return intake subsystem
     */
    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
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
