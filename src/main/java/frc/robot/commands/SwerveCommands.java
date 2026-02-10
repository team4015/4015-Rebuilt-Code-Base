package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveCommands extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier turningSpeedSupplier;
    public final BooleanSupplier fieldOrientedSupplier;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(5);

    public SwerveCommands(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, DoubleSupplier turningSpeedSupplier, BooleanSupplier fieldOrientdSupplier){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;
        this.fieldOrientedSupplier = fieldOrientdSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.getAsDouble(), Constants.OIConstants.kDeadband);
        double ySpeed = MathUtil.applyDeadband(ySpeedSupplier.getAsDouble(), Constants.OIConstants.kDeadband);
        double turningSpeed = MathUtil.applyDeadband(turningSpeedSupplier.getAsDouble(), Constants.OIConstants.kDeadband);

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = xLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = xLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = fieldOrientedSupplier.getAsBoolean() ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModulesStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

}
