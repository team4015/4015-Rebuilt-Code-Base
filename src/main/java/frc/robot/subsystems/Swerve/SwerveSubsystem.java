/*************************************************************************************************
 @Name: Gursahaj Chawla
 @Date: 2/10/2026
 @File: SwerveSubsystem.java
 @Description: This class in the program and the setup for the whole Swerve Subsystem in which
 all the methods and created inorder for them to be used to execute the program using the
 Command code from SwerveCommands.java
 ***********************************************************************************************/

package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    //creates four SwerveModule objects with all the parameters added
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.frontLeftDriveMotorPort,
        DriveConstants.frontLeftTurningMotorPort,
        DriveConstants.frontLeftDriveEncoderReversed,
        DriveConstants.frontLeftTurningEncoderReversed,
        DriveConstants.frontLeftDriveAbsoluteEncoderPort,
        DriveConstants.frontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.frontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.frontRightDriveMotorPort,
        DriveConstants.frontRightTurningMotorPort,
        DriveConstants.frontRightDriveEncoderReversed,
        DriveConstants.frontRightTurningEncoderReversed,
        DriveConstants.frontRightDriveAbsoluteEncoderPort,
        DriveConstants.frontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.frontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.backLeftDriveMotorPort,
        DriveConstants.backLeftTurningMotorPort,
        DriveConstants.backLeftDriveEncoderReversed,
        DriveConstants.backLeftTurningEncoderReversed,
        DriveConstants.backLeftDriveAbsoluteEncoderPort,
        DriveConstants.backLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.backLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.backRightDriveMotorPort,
        DriveConstants.backRightTurningMotorPort,
        DriveConstants.backRightDriveEncoderReversed,
        DriveConstants.backRightTurningEncoderReversed,
        DriveConstants.backRightDriveAbsoluteEncoderPort,
        DriveConstants.backRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.backRightDriveAbsoluteEncoderReversed
    );


    //Create an object and initialize the NavX gyro
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    //The SwerveDriveOdometry class tracks the robot's pose (position) on the field as x, y coordinates (meters) and rotation
    private final SwerveDriveOdometry odometer= new SwerveDriveOdometry(
        DriveConstants.driveKinematics, //SwerveDriveKinematic Object which helps tell the geometry of the robot
        Rotation2d.fromDegrees(0), //set initial heading to 0
        //This is an array of initial position of each swerve module and returns the distance driven and angle
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );

    //The constructor create a Thread to sleep the program
    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); //call the method
            } catch (Exception e) {
            }
        }).start(); 
    }

    //This method is a void method to reset the gyro heading to 0
    public void zeroHeading(){
        gyro.reset();
    }

    //This gets the heading angle between 0-360 (related acute angle from -π to π)
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //Get the WPILid normalized angle from the heading angle
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    //gets the positions of the robot in (x,y,z)
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    //This resets the position of the robot
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), 
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }, pose
        );
    }

    //This periodic() method updates the odometer of the swerve module to get the updates position of the swerve modules
    @Override
    public void periodic(){
        odometer.update(getRotation2d(), new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("Absolute Encoder Front Left", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder Front Right", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder Back Left", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder Back Right", backRight.getAbsoluteEncoderRad());

        SmartDashboard.putNumber("Turning Encoder Front Left", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Turning Encoder Front Right", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Turning Encoder Back Left", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Turning Encoder Back Right", backRight.getTurningPosition());
    }

    //stop all the swerve modules
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //This sets the brake mode of the robot, if false, the brakes on the motor are off, if true the motor's brakes are on
    public void setBrakeMode(boolean enabled){
        frontLeft.setBrakeMode(enabled);
        frontRight.setBrakeMode(enabled);
        backLeft.setBrakeMode(enabled);
        backRight.setBrakeMode(enabled);
    }

    //Gets the SwerveModuleStates (explained before)
    public void setModulesStates(SwerveModuleState[] desiredStates){
        //Desaturated scales all wheel speeds proportional so that the fastest wheel is the max speed
        //This is to limit the kinematics producing wheelSpeeds higher than the physical speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.physicalMaxSpeedMetersPerSecond);
        //Set the desired States for each by reducing it by a scaled factor
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public double getFrontLeftAbsoluteEncoderRad(){
        return frontLeft.getAbsoluteEncoderRad();
    }

    public double getFrontRightAbsoluteEncoderRad(){
        return frontRight.getAbsoluteEncoderRad();
    }

    public double getBackRightAbsoluteEncoderRad(){
        return backRight.getAbsoluteEncoderRad();
    }

    public double getBackLeftAbsoluteEncoderRad(){
        return backLeft.getAbsoluteEncoderRad();
    }
}
