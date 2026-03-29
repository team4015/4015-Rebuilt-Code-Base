package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.MoveIntakeArmCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.IndexerCommand;
import frc.robot.commands.Shooter.PresetShootCommand;
import frc.robot.commands.Swerve.SwerveCommands;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * Builds the robot's subsystems, commands, and controller bindings.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(swerveSubsystem, limelightSubsystem);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final Joystick driverJoystick = new Joystick(Constants.OIConstants.driverControllerPort);
    private final Joystick operatorJoystick = new Joystick(Constants.OIConstants.operatorControllerPort);

    private final PathPlannerAuto auto;
    private final SendableChooser<Command> autoChooser;

    /**
     * Creates the robot container and installs default commands and button bindings.
     */
    public RobotContainer() {
    NamedCommands.registerCommand("intake", new IntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand("extendIntake", new MoveIntakeArmCommand(intakeSubsystem, MoveIntakeArmCommand.Direction.DOWN));
    NamedCommands.registerCommand("shoot", new ShooterCommand(shooterSubsystem));
    NamedCommands.registerCommand("index", new IndexerCommand(shooterSubsystem));

    auto = new PathPlannerAuto("rebuiltAuto");
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
        swerveSubsystem.setDefaultCommand(
            new SwerveCommands(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.driverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverXAxis), //change with negative sign
                () -> driverJoystick.getRawAxis(OIConstants.driverRotAxis), //change with negative sign
                // Default to robot-centric driving for consistent joystick forward = robot forward.
                () -> !driverJoystick.getRawButton(OIConstants.driverFieldOrientedButtonIdx),
                () -> driverJoystick.getRawButton(OIConstants.aimAtTagButtonIdx),
                limelightSubsystem,
                shooterSubsystem
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        bindOnTrue(OIConstants.shooterToggleButtonIdx, new ShooterCommand(shooterSubsystem));
        bindOnTrue(OIConstants.presetShootButtonIdx, new PresetShootCommand(shooterSubsystem, limelightSubsystem));
        bindOnTrue(OIConstants.intakeToggleButtonIdx, new IntakeCommand(intakeSubsystem));
        new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem));
        // Intake arm: press to drive down until limit; hold to drive up manually.
        new JoystickButton(operatorJoystick, OIConstants.extendIntakeToggleButtonIdx)
            .onTrue(new MoveIntakeArmCommand(intakeSubsystem, MoveIntakeArmCommand.Direction.DOWN));
        new JoystickButton(operatorJoystick, OIConstants.retractIntakeToggleButtonIdx)
            .whileTrue(new MoveIntakeArmCommand(intakeSubsystem, MoveIntakeArmCommand.Direction.UP));
    }

    private void bindOnTrue(int buttonIdx, Command command) {
        new JoystickButton(operatorJoystick, buttonIdx).onTrue(command);
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
        return auto;
    }
}
