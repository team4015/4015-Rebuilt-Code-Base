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

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry odometer= new SwerveDriveOdometry(
        DriveConstants.driveKinematics,
        Rotation2d.fromDegrees(0),
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );

    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start(); 
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

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
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModulesStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
