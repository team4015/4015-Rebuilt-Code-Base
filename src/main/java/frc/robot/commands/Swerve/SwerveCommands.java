package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;


/**
 * Default teleop drive command for the swerve subsystem.
 *
 * <p>Reads joystick inputs, applies deadband and slew limiting, scales to configured speed limits,
 * then converts desired chassis motion into module states.</p>
 */
public class SwerveCommands extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier turningSpeedSupplier;
    private final BooleanSupplier fieldOrientedSupplier;
    private final BooleanSupplier autoAimSupplier;
    private final LimelightSubsystem limelightSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    // Rate limiters smooth operator input and reduce jerk.
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.DriveConstants.teleDriveAngularAccelerationUnitsPerSecond);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(Constants.DriveConstants.teleDriveAngularAccelerationUnitsPerSecond);

    /**
     * Creates the swerve teleop command.
     *
     * @param swerveSubsystem subsystem controlled by this command
     * @param xSpeedSupplier supplier for forward/backward joystick input
     * @param ySpeedSupplier supplier for left/right joystick input
     * @param turningSpeedSupplier supplier for rotational joystick input
     * @param fieldOrientdSupplier supplier for field-oriented enable state
     * @param autoAimSupplier supplier for whether Limelight auto-aim is enabled
     * @param limelightSubsystem Limelight interface used for AprilTag aiming and distance
     * @param shooterSubsystem shooter interface used for motion-compensated aiming while firing
     */
    public SwerveCommands(
        SwerveSubsystem swerveSubsystem,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier turningSpeedSupplier,
        BooleanSupplier fieldOrientdSupplier,
        BooleanSupplier autoAimSupplier,
        LimelightSubsystem limelightSubsystem,
        ShooterSubsystem shooterSubsystem
    )    {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;
        this.fieldOrientedSupplier = fieldOrientdSupplier;
        this.autoAimSupplier = autoAimSupplier;
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(swerveSubsystem);
    }

    /**
     * Performs one teleop control update and writes desired module states.
     */
    @Override
    public void execute() {
        double rawXSpeed = xSpeedSupplier.getAsDouble();
        double rawYSpeed = ySpeedSupplier.getAsDouble();
        double rawTurningSpeed = turningSpeedSupplier.getAsDouble();

        // Remove small joystick noise around center.
        double xSpeed = MathUtil.applyDeadband(rawXSpeed, Constants.OIConstants.deadband);
        double ySpeed = MathUtil.applyDeadband(rawYSpeed, Constants.OIConstants.deadband);
        double turningSpeed = MathUtil.applyDeadband(rawTurningSpeed, Constants.OIConstants.deadband);

        // Smooth and scale inputs into physical command units.
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.teleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.teleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

        boolean autoAimEnabled = autoAimSupplier.getAsBoolean() || shooterSubsystem.isShootingActive();
        boolean hasVisionTarget = limelightSubsystem.hasValidTarget();
        if (autoAimEnabled && hasVisionTarget) {
            turningSpeed = limelightSubsystem.getAimAngularSpeedRadPerSec();
        }

        swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, fieldOrientedSupplier.getAsBoolean());
    }

    /**
     * Stops the swerve modules when command ends.
     *
     * @param interrupted whether the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
