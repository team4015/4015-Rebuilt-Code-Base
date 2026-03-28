package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.PresetShootCommand;
import frc.robot.commands.Swerve.SwerveCommands;
import frc.robot.commands.FlopOver.FlopOverCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;
import frc.robot.subsystems.FlopOver.FlopOverSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Builds the robot's subsystems, commands, and controller bindings.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(swerveSubsystem, limelightSubsystem);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FlopOverSubsystem flopOverSubsystem = new FlopOverSubsystem();
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
        configureButtonBindings();
    }

    private void configureBindings() {
        bindOnTrue(OIConstants.shooterToggleButtonIdx, new ShooterCommand(shooterSubsystem));
        bindOnTrue(OIConstants.presetShootButtonIdx, new PresetShootCommand(shooterSubsystem, limelightSubsystem));
        bindOnTrue(OIConstants.intakeToggleButtonIdx, new IntakeCommand(intakeSubsystem));
        // Button X deploys the flop-over arm (intake tilt) until the down limit switch trips.
        bindOnTrue(OIConstants.flopOverButtonIdx, new FlopOverCommand(flopOverSubsystem));
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem));
        // // D-pad snap to field-relative cardinals (0/90/180/270 deg).
        // new POVButton(driverJoystick, 0).onTrue(new SnapHeadingCommand(swerveSubsystem, Rotation2d.fromDegrees(0)));
        // new POVButton(driverJoystick, 90).onTrue(new SnapHeadingCommand(swerveSubsystem, Rotation2d.fromDegrees(90)));
        // new POVButton(driverJoystick, 180).onTrue(new SnapHeadingCommand(swerveSubsystem, Rotation2d.fromDegrees(180)));
        // new POVButton(driverJoystick, 270).onTrue(new SnapHeadingCommand(swerveSubsystem, Rotation2d.fromDegrees(270)));

        // Auto-align button also spins up shooter, then indexer; releases stop both.
        new JoystickButton(driverJoystick, OIConstants.aimAtTagButtonIdx)
            .onTrue(new PresetShootCommand(shooterSubsystem, limelightSubsystem))
            .onFalse(new InstantCommand(shooterSubsystem::stopAll, shooterSubsystem));
    }

    private void bindOnTrue(int buttonIdx, Command command) {
        new JoystickButton(driverJoystick, buttonIdx).onTrue(command);
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
