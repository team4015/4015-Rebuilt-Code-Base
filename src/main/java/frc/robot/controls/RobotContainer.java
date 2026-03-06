package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Central wiring class for subsystems, default commands, and button bindings.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Joystick driverJoystick = new Joystick(Constants.OIConstants.driverControllerPort);

    /**
     * Constructs container and configures defaults/bindings.
     */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(
            new SwerveCommands(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.driverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverRotAxis),
                // Hold button to enable field-oriented mode; default is robot-oriented.
                () -> driverJoystick.getRawButton(OIConstants.driverFieldOrientedButtonIdx)
            )
        );

        configureBindings();
    }

    /**
     * Configures operator button mappings.
     *
     * <p>Unlock button temporarily sets coast mode and stops modules while held, then restores
     * brake mode when released.</p>
     */
    private void configureBindings() {
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
     * @return primary swerve subsystem
     */
    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    /**
     * @return autonomous command to schedule, or {@code null} if none configured
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
