package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Swerve drivetrain subsystem with odometry and encoder-calibration utilities.
 */
public class SwerveSubsystem extends SubsystemBase {
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

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.driveKinematics,
        Rotation2d.fromDegrees(0),
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );

    /** Creates the swerve subsystem */
    public SwerveSubsystem() {
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){

            }
        }).start();
    }

    /** Resets the NavX heading to zero. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the robot heading in degrees.
     *
     * @return wrapped heading in degrees
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360.0);
    }

    /**
     * Returns the robot heading as a {@link Rotation2d}.
     *
     * @return robot rotation
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Returns the current estimated robot pose.
     *
     * @return current pose in meters
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Returns measured robot-relative chassis speeds derived from the modules.
     *
     * @return robot-relative chassis speeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.driveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    /**
     * Resets odometry to the specified pose.
     *
     * @param pose new pose estimate
     */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    @Override
    public void periodic() {
        odometer.update(
            getRotation2d(),
            new SwerveModulePosition[]{
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

    /** Re-synchronizes all module encoders to their absolute angles and zeroes drive distances. */
    public void resetModuleEncoders() {
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    /** Stops all four swerve modules. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets brake or coast mode on all modules.
     *
     * @param enabled {@code true} for brake mode, {@code false} for coast mode
     */
    public void setBrakeMode(boolean enabled) {
        frontLeft.setBrakeMode(enabled);
        frontRight.setBrakeMode(enabled);
        backLeft.setBrakeMode(enabled);
        backRight.setBrakeMode(enabled);
    }

    /**
     * Applies desired states to all modules after wheel-speed desaturation.
     *
     * @param desiredStates desired module states in standard order
     */
    public void setModulesStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Drives the robot using chassis-speed commands.
     *
     * @param xSpeedMetersPerSecond forward speed
     * @param ySpeedMetersPerSecond sideways speed
     * @param omegaRadiansPerSecond angular speed
     * @param fieldRelative whether the translation commands are field-relative
     */
    public void drive(
        double xSpeedMetersPerSecond,
        double ySpeedMetersPerSecond,
        double omegaRadiansPerSecond,
        boolean fieldRelative
    ) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedMetersPerSecond,
                ySpeedMetersPerSecond,
                omegaRadiansPerSecond,
                getRotation2d()
            )
            : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, omegaRadiansPerSecond);

        setModulesStates(DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Returns the front-left corrected absolute encoder angle.
     *
     * @return angle in radians
     */
    public double getFrontLeftAbsoluteEncoderRad() {
        return frontLeft.getAbsoluteEncoderRad();
    }

    /**
     * Returns the front-right corrected absolute encoder angle.
     *
     * @return angle in radians
     */
    public double getFrontRightAbsoluteEncoderRad() {
        return frontRight.getAbsoluteEncoderRad();
    }

    /**
     * Returns the back-right corrected absolute encoder angle.
     *
     * @return angle in radians
     */
    public double getBackRightAbsoluteEncoderRad() {
        return backRight.getAbsoluteEncoderRad();
    }

    /**
     * Returns the back-left corrected absolute encoder angle.
     *
     * @return angle in radians
     */
    public double getBackLeftAbsoluteEncoderRad() {
        return backLeft.getAbsoluteEncoderRad();
    }
}
