package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.FlopOverCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.PresetShootCommand;
import frc.robot.commands.Swerve.SwerveCommands;
import frc.robot.subsystems.Intake.FlopOverSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
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
        bindOnTrue(OIConstants.flopToggleButtonIdx, new FlopOverCommand(flopOverSubsystem));
    }

    private void configureButtonBindings() {
        // Rebind zero-heading to an unused stick press to keep A/B free for intake controls.
        new JoystickButton(driverJoystick, 9).onTrue(new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem));
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
