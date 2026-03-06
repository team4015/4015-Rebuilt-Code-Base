package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

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

    // Rate limiters smooth operator input and reduce jerk.
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(5);

    /**
     * Creates the swerve teleop command.
     *
     * @param swerveSubsystem subsystem controlled by this command
     * @param xSpeedSupplier supplier for forward/backward joystick input
     * @param ySpeedSupplier supplier for left/right joystick input
     * @param turningSpeedSupplier supplier for rotational joystick input
     * @param fieldOrientdSupplier supplier for field-oriented enable state
     */
    public SwerveCommands(
        SwerveSubsystem swerveSubsystem,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier turningSpeedSupplier,
        BooleanSupplier fieldOrientdSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;
        this.fieldOrientedSupplier = fieldOrientdSupplier;
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
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * Constants.DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = fieldOrientedSupplier.getAsBoolean()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModulesStates(moduleStates);

        SmartDashboard.putNumber("OI Raw X", rawXSpeed);
        SmartDashboard.putNumber("OI Raw Y", rawYSpeed);
        SmartDashboard.putNumber("OI Raw Rot", rawTurningSpeed);
        SmartDashboard.putBoolean("OI Field Oriented", fieldOrientedSupplier.getAsBoolean());
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
}
