package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Rotates the robot to a specified field-relative heading using the gyro.
 */
public class SnapHeadingCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController controller = new PIDController(DriveConstants.snapHeadingKp, 0.0, 0.0);
    private final Rotation2d targetHeading;

    public SnapHeadingCommand(SwerveSubsystem swerveSubsystem, Rotation2d targetHeading) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetHeading = targetHeading;
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(DriveConstants.snapHeadingToleranceRad);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        double current = MathUtil.angleModulus(swerveSubsystem.getRotation2d().getRadians());
        double setpoint = MathUtil.angleModulus(targetHeading.getRadians());
        double output = controller.calculate(current, setpoint);
        double clampedOmega = MathUtil.clamp(
            output,
            -DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond,
            DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond
        );
        swerveSubsystem.drive(0.0, 0.0, clampedOmega, true);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
