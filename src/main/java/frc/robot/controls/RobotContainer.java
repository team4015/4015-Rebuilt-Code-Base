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
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.ExtendableHopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ExtendableHopperSubsystem extendableHopperSubsystem = new ExtendableHopperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final Joystick driverJoystick = new Joystick(Constants.OIConstants.driverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(
            new SwerveCommands(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.driverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.driverRotAxis),
                () -> driverJoystick.getRawButton(OIConstants.driverFieldOrientedButtonIdx),
                () -> driverJoystick.getRawButton(OIConstants.aimAtTagButtonIdx),
                limelightSubsystem
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, OIConstants.intakeToggleButtonIdx)
            .onTrue(new IntakeCommand(intakeSubsystem));

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

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    public LimelightSubsystem getLimelightSubsystem() {
        return limelightSubsystem;
    }

    public ExtendableHopperSubsystem getExtendableHopperSubsystem() {
        return extendableHopperSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
