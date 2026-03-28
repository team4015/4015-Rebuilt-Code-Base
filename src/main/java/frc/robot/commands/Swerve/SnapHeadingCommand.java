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
    private static final double kP = 0.5; // tuned to be responsive; capped below
    private static final double ANGLE_TOLERANCE_RAD = Math.toRadians(2.0);

    private final SwerveSubsystem swerveSubsystem;
    private final PIDController controller = new PIDController(kP, 0.0, 0.0);
    private final Rotation2d targetHeading;

    public SnapHeadingCommand(SwerveSubsystem swerveSubsystem, Rotation2d targetHeading) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetHeading = targetHeading;
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(ANGLE_TOLERANCE_RAD);
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
