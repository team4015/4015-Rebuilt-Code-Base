package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class SwerveSubsystem extends SubsystemBase {

    double maximumSpeed = 4.75;


    SwerveDrive swerveDrive;
    double deadband = Constants.OperatorConstants.DEADBAND;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(3);        

    public SwerveSubsystem(File directory) {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);

        } catch (IOException e) {
            throw new RuntimeException("Swerve Drive directory not found. Finish those config files!");
        }

        swerveDrive.zeroGyro();
        
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command. NOT BEING USED */
     
    /*public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {

        return run(() -> {

            Translation2d rawInputs = new Translation2d(translationX.getAsDouble(), translationY.getAsDouble());
            Translation2d scaledInputs = SwerveMath.scaleTranslation(rawInputs, 0.8);

            // Make the robot move

            ChassisSpeeds chassisSpeeds = swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.getX(),
                    scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity());

            swerveDrive.driveFieldOriented(chassisSpeeds);
        });
    } 

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    private boolean fieldOriented = false;

    public void toggleFieldOriented(){
        fieldOriented = !fieldOriented;
    }

    public boolean isFieldOriented(){
        return fieldOriented;
    }
    
    
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
            
        System.out.println("Reached driveCommand");

        return run(() -> {
            double maxVelocity = swerveDrive.getMaximumChassisVelocity();
            
            //Getting the doublee from the double suppliers and multiplying them by their max velocity
            double vxMetersPerSecond = MathUtil.applyDeadband(translationX.getAsDouble(), deadband);
            double vyMetersPerSecond = MathUtil.applyDeadband(translationY.getAsDouble(), deadband);
            double angularRotation = MathUtil.applyDeadband(angularRotationX.getAsDouble(), deadband + 0.075);
            
            //Limit the slew rate of the speeds to prevent sporadic motion and apply a deadband
            
            vxMetersPerSecond = xLimiter.calculate(vxMetersPerSecond* maxVelocity) ;
            vyMetersPerSecond = yLimiter.calculate(vyMetersPerSecond* maxVelocity) ;
            angularRotation = turningLimiter.calculate(angularRotation * swerveDrive.getMaximumChassisAngularVelocity());
            
            if (Math.abs(angularRotation) < 0.1){
                angularRotation = 0;
            }
            // Make the robot move
            
            //Rotation2d angularHeading = swerveDrive.getOdometryHeading();
            //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vxMetersPerSecond,vyMetersPerSecond,angularRotation);
            //ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, angularHeading);

            Translation2d translation = new Translation2d(vxMetersPerSecond,vyMetersPerSecond);
            
            
            swerveDrive.drive(translation, angularRotation, fieldOriented, false);
            
            //swerveDrive.driveFieldOriented(fieldRelativeSpeeds);

            System.out.println("In Run Method" + "vxMetersPerSecond:" + vxMetersPerSecond + "vyMetersPerSecond: " + vyMetersPerSecond);
            //swerveDrive.drive(translation, angularRotation, true, false);
            //ChassisSpeeds.fromFieldRelativeSpeeds(null, null);
        });


    }

    public SwerveDrive getSwerveDrive(){
        return swerveDrive;
    }

}