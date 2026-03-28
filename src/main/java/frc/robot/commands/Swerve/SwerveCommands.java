package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final BooleanSupplier autoAimSupplier;
    private final LimelightSubsystem limelightSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    // Rate limiters smooth operator input and reduce jerk.
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    /**
     * Creates the swerve teleop command.
     *
     * @param swerveSubsystem subsystem controlled by this command
     * @param xSpeedSupplier supplier for forward/backward joystick input
     * @param ySpeedSupplier supplier for left/right joystick input
     * @param omegaSpeedSupplier supplier for rotational joystick input
     * @param fieldOrientdSupplier supplier for field-oriented enable state
     * @param autoAimSupplier supplier for whether Limelight auto-aim is enabled
     * @param limelightSubsystem Limelight interface used for AprilTag aiming and distance
     * @param shooterSubsystem shooter interface used for motion-compensated aiming while firing
     */
    public SwerveCommands(
            SwerveSubsystem swerveSubsystem
            Supplier<Double> xSpeedFunction,
            Supplier<Double>ySpeedFunction,
            Supplier<Double>turningSpeedFunction;
            Supplier<Boolean> fieldOrientedFunction;
            BooleanSupplier autoAimSupplier;
            LimelightSubsystem limelightSubsystem;
            ShooterSubsystem shooterSubsystem;
    )    {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turningSpeedFunction = turningSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter((Constants.DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond));
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
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turningSpeed = turningSpeedFunction.get();

        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.deadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.deadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.deadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.teleDriveMaxAccelerationUnitsPerSecond;
        ySpeed = xLimiter.calculate(ySpeed) * Constants.DriveConstants.teleDriveMaxAccelerationUnitsPerSecond;
        turningSpeed = xLimiter.calculate(turningSpeed) * Constants.DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;

        if(fieldOrientedFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }



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
