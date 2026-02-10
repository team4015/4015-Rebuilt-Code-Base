/*************************************************************************************************
 @Name: Gursahaj Chawla
 @Date: 2/10/2026
 @File: SwerveCommands.java
 @Description: This class in the program to create the commands for the robot in joystick
 by create the command-based class
 ***********************************************************************************************/

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
    private final SwerveSubsystem swerveSubsystem; //create Reference variables for the swerveSubsystem
    //The DoubleSupplier is a functional interface that supplies double whenever it's called
    private final DoubleSupplier xSpeedSupplier; //represents forward and backward motion
    private final DoubleSupplier ySpeedSupplier;//represents left and right motion
    private final DoubleSupplier turningSpeedSupplier; //represents the turning motion. + --> clockwise, --> counterclockwise
    public final BooleanSupplier fieldOrientedSupplier; //tell if this is field oriented

    //This is used to smooth out the data curve to remove high jerk in the robot for driving and turning
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(5);

    //Set constructor with the parameters from the variable above not including the SlewRateLimiter objects
    public SwerveCommands(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, DoubleSupplier turningSpeedSupplier, BooleanSupplier fieldOrientdSupplier){
        //create instances of the objects
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;
        this.fieldOrientedSupplier = fieldOrientdSupplier;
        addRequirements(swerveSubsystem); //add the swerveSubsystem as a requirement
    }

    @Override
    public void execute(){
        //set the speeds to the Suppliers and applies the deadbands to the joystick for all the components
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.getAsDouble(), Constants.OIConstants.deadband);
        double ySpeed = MathUtil.applyDeadband(ySpeedSupplier.getAsDouble(), Constants.OIConstants.deadband);
        double turningSpeed = MathUtil.applyDeadband(turningSpeedSupplier.getAsDouble(), Constants.OIConstants.deadband);

        //reinitialize the variable with the SlewRateLimiter and also adding the max speed of a robot in teleoperidic
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.teleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.teleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

        //ChassisSpeeds represents the robot-level movement and has three components (vx --> forward/backwards,vy --> left/right, vw --> rotationalSpeed)
        //The ternary operator is used then to see if its field oriented, if true, it will use the relative field speed and if not uses the robot relative speed
        ChassisSpeeds chassisSpeeds = fieldOrientedSupplier.getAsBoolean() ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        //Set the moduleStates for each of the swerveModules and the drive Kinematics
        SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModulesStates(moduleStates);
    }

    //when the program ends all the swerveModules stop
    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

}
