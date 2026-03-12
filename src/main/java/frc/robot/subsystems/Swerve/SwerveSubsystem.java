package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Swerve drivetrain subsystem with odometry and encoder-calibration utilities.
 */
public class SwerveSubsystem extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;

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

    private final SwerveModule[] modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
    private final String[] moduleNames = new String[]{"Front Left", "Front Right", "Back Left", "Back Right"};
    private final double[] calibrationRawSinSum = new double[4];
    private final double[] calibrationRawCosSum = new double[4];
    private int calibrationSampleCount = 0;

    private GenericEntry calCollectEntry;
    private GenericEntry calResetEntry;
    private GenericEntry calTargetDegEntry;
    private GenericEntry calSampleCountEntry;

    private final GenericEntry[] rawNowDegEntries = new GenericEntry[4];
    private final GenericEntry[] rawAvgDegEntries = new GenericEntry[4];
    private final GenericEntry[] correctedDegEntries = new GenericEntry[4];
    private final GenericEntry[] turnAbsErrorDegEntries = new GenericEntry[4];
    private final GenericEntry[] configOffsetRadEntries = new GenericEntry[4];
    private final GenericEntry[] recommendedOffsetDegEntries = new GenericEntry[4];
    private final GenericEntry[] recommendedOffsetRadEntries = new GenericEntry[4];
    private final GenericEntry[] absoluteReversedEntries = new GenericEntry[4];
    private final GenericEntry[] turningMotorReversedEntries = new GenericEntry[4];
    private final GenericEntry[] driveMotorReversedEntries = new GenericEntry[4];
    private final GenericEntry[] absTurnSignMatchEntries = new GenericEntry[4];
    private final GenericEntry[] turnAppliedSignMatchEntries = new GenericEntry[4];
    private final GenericEntry[] driveAppliedSignMatchEntries = new GenericEntry[4];
    private final GenericEntry[] rawDeltaDegEntries = new GenericEntry[4];
    private final GenericEntry[] turningAppliedEntries = new GenericEntry[4];
    private final GenericEntry[] driveAppliedEntries = new GenericEntry[4];
    private final GenericEntry[] turningVelocityEntries = new GenericEntry[4];
    private final GenericEntry[] driveVelocityEntries = new GenericEntry[4];
    private final GenericEntry[] channelEntries = new GenericEntry[4];
    private final double[] previousRawAbsoluteRad = new double[4];
    private final boolean[] previousRawInitialized = new boolean[4];

    // Delay heading zeroing until the NavX finishes its startup calibration to avoid
    // noisy yaw values that make field-oriented driving feel random.
    private static final double HEADING_INIT_TIMEOUT_SEC = 5.0;
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final Timer startupTimer = new Timer();
    private boolean headingInitialized = false;

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

    /** Creates the swerve subsystem and initializes calibration telemetry. */
    public SwerveSubsystem() {
        initializeCalibrationDashboard();
        startupTimer.start();
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
        return Rotation2d.fromDegrees(-getHeading());
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
        if (!headingInitialized) {
            boolean gyroReady = !gyro.isCalibrating();
            boolean timeoutReached = startupTimer.hasElapsed(HEADING_INIT_TIMEOUT_SEC);
            if (gyroReady || timeoutReached) {
                zeroHeading();
                resetOdometry(new Pose2d());
                headingInitialized = true;
            }
        }

        updateCalibrationSampling();
        odometer.update(
            getRotation2d(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        publishCalibrationTelemetry();

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

    /**
     * Creates and wires the Shuffleboard widgets used for encoder calibration.
     *
     * <p>The dashboard contains controls for sample collection and a module-specific
     * telemetry panel for raw/corrected angles, inversion status, command/velocity sign checks,
     * and recommended offsets.
     */
    private void initializeCalibrationDashboard() {
        ShuffleboardTab calibrationTab = Shuffleboard.getTab("Encoder Calibration");

        ShuffleboardLayout controlsLayout = calibrationTab
            .getLayout("Controls", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 6);

        calTargetDegEntry = controlsLayout
            .add("Target Angle (deg)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        calCollectEntry = controlsLayout
            .add("Collect Samples", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
        calResetEntry = controlsLayout
            .add("Reset Samples", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        calSampleCountEntry = controlsLayout
            .add("Sample Count", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        controlsLayout
            .add(
                "Instructions",
                "Point wheels to target angle, reset samples once, collect 150+ samples, copy Recommended Offset (rad) to driveConfig.json"
            )
            .withWidget(BuiltInWidgets.kTextView);
        controlsLayout
            .add(
                "Inversion Guide",
                "While driving slowly on blocks: SignMatch fields should be true when module motion is above threshold."
            )
            .withWidget(BuiltInWidgets.kTextView);

        for (int i = 0; i < modules.length; i++) {
            int col = 2 + (i % 2) * 4;
            int row = (i / 2) * 6;
            ShuffleboardLayout moduleLayout = calibrationTab
                .getLayout(moduleNames[i], BuiltInLayouts.kList)
                .withPosition(col, row)
                .withSize(4, 6);

            rawNowDegEntries[i] = moduleLayout.add("Raw Now (deg)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            rawAvgDegEntries[i] = moduleLayout.add("Raw Avg (deg)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            correctedDegEntries[i] = moduleLayout.add("Corrected (deg)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            turnAbsErrorDegEntries[i] = moduleLayout.add("Turn-Abs Error (deg)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            configOffsetRadEntries[i] = moduleLayout.add("Config Offset (rad)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            recommendedOffsetDegEntries[i] = moduleLayout.add("Recommended Offset (deg)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            recommendedOffsetRadEntries[i] = moduleLayout.add("Recommended Offset (rad)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            absoluteReversedEntries[i] = moduleLayout.add("Absolute Reversed", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            turningMotorReversedEntries[i] = moduleLayout.add("Turning Motor Reversed", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            driveMotorReversedEntries[i] = moduleLayout.add("Drive Motor Reversed", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            rawDeltaDegEntries[i] = moduleLayout.add("Raw Delta/Loop (deg)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            turningAppliedEntries[i] = moduleLayout.add("Turning Applied", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            driveAppliedEntries[i] = moduleLayout.add("Drive Applied", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            turningVelocityEntries[i] = moduleLayout.add("Turning Vel (rad/s)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            driveVelocityEntries[i] = moduleLayout.add("Drive Vel (m/s)", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
            absTurnSignMatchEntries[i] = moduleLayout.add("Abs vs Turn Sign Match", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            turnAppliedSignMatchEntries[i] = moduleLayout.add("Turn Cmd vs Vel Match", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            driveAppliedSignMatchEntries[i] = moduleLayout.add("Drive Cmd vs Vel Match", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            channelEntries[i] = moduleLayout.add("Encoder Channel", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        }
    }

    /**
     * Processes calibration control inputs from Shuffleboard.
     *
     * <p>If reset is requested, sample buffers are cleared. If collection is enabled,
     * one sample is captured from each module on this loop iteration.
     */
    private void updateCalibrationSampling() {
        if (calResetEntry.getBoolean(false)) {
            resetCalibrationSamples();
            calResetEntry.setBoolean(false);
        }
        if (calCollectEntry.getBoolean(false)) {
            captureCalibrationSample();
        }
    }

    /** Resets the accumulated calibration sample count and circular angle averages. */
    private void resetCalibrationSamples() {
        calibrationSampleCount = 0;
        for (int i = 0; i < calibrationRawSinSum.length; i++) {
            calibrationRawSinSum[i] = 0.0;
            calibrationRawCosSum[i] = 0.0;
        }
    }

    /** Captures one raw absolute-angle sample from each module into circular-average accumulators. */
    private void captureCalibrationSample() {
        for (int i = 0; i < modules.length; i++) {
            double rawRad = modules[i].getRawAbsoluteEncoderRad();
            calibrationRawSinSum[i] += Math.sin(rawRad);
            calibrationRawCosSum[i] += Math.cos(rawRad);
        }
        calibrationSampleCount++;
    }

    /**
     * Publishes calibration and inversion telemetry for each module to Shuffleboard.
     *
     * <p>This method computes real-time raw/corrected angles, average sampled raw angle,
     * offset recommendations, and command-versus-feedback sign checks used during bring-up.
     */
    private void publishCalibrationTelemetry() {
        calSampleCountEntry.setInteger(calibrationSampleCount);
        double targetDeg = calTargetDegEntry.getDouble(0.0);
        double targetRad = Math.toRadians(targetDeg);

        for (int i = 0; i < modules.length; i++) {
            double rawNowRad = modules[i].getRawAbsoluteEncoderRad();
            double rawDeltaRad = 0.0;
            if (previousRawInitialized[i]) {
                rawDeltaRad = MathUtil.angleModulus(rawNowRad - previousRawAbsoluteRad[i]);
            } else {
                previousRawInitialized[i] = true;
            }
            previousRawAbsoluteRad[i] = rawNowRad;

            double avgRawRad = calibrationSampleCount > 0
                ? getAverageRawAngleRad(i)
                : rawNowRad;
            double correctedRad = modules[i].getAbsoluteEncoderRad();
            double turnAbsErrorRad = modules[i].normalizeTurningAngle(modules[i].getTurningPosition() - correctedRad);
            double recommendedOffsetRad = calculateRecommendedOffsetRad(
                avgRawRad,
                targetRad,
                modules[i].isAbsoluteEncoderReversed()
            );
            double turningApplied = modules[i].getTurningAppliedOutput();
            double driveApplied = modules[i].getDriveAppliedOutput();
            double turningVelocity = modules[i].getTurningVelocity();
            double driveVelocity = modules[i].getDriveVelocity();
            boolean absTurnSignMatch = signsMatchWithThreshold(rawDeltaRad, turningVelocity, Math.toRadians(0.08), 0.10);
            boolean turnCmdSignMatch = signsMatchWithThreshold(turningApplied, turningVelocity, 0.05, 0.10);
            boolean driveCmdSignMatch = signsMatchWithThreshold(driveApplied, driveVelocity, 0.05, 0.08);

            rawNowDegEntries[i].setDouble(Math.toDegrees(rawNowRad));
            rawAvgDegEntries[i].setDouble(Math.toDegrees(avgRawRad));
            correctedDegEntries[i].setDouble(Math.toDegrees(correctedRad));
            turnAbsErrorDegEntries[i].setDouble(Math.toDegrees(turnAbsErrorRad));
            configOffsetRadEntries[i].setDouble(modules[i].getAbsoluteEncoderOffsetRad());
            recommendedOffsetDegEntries[i].setDouble(Math.toDegrees(recommendedOffsetRad));
            recommendedOffsetRadEntries[i].setDouble(recommendedOffsetRad);
            absoluteReversedEntries[i].setBoolean(modules[i].isAbsoluteEncoderReversed());
            turningMotorReversedEntries[i].setBoolean(modules[i].isTurningMotorReversed());
            driveMotorReversedEntries[i].setBoolean(modules[i].isDriveMotorReversed());
            rawDeltaDegEntries[i].setDouble(Math.toDegrees(rawDeltaRad));
            turningAppliedEntries[i].setDouble(turningApplied);
            driveAppliedEntries[i].setDouble(driveApplied);
            turningVelocityEntries[i].setDouble(turningVelocity);
            driveVelocityEntries[i].setDouble(driveVelocity);
            absTurnSignMatchEntries[i].setBoolean(absTurnSignMatch);
            turnAppliedSignMatchEntries[i].setBoolean(turnCmdSignMatch);
            driveAppliedSignMatchEntries[i].setBoolean(driveCmdSignMatch);
            channelEntries[i].setInteger(modules[i].getAbsoluteEncoderChannel());
        }
    }

    /**
     * Compares signal signs only when both signals exceed configured deadbands.
     *
     * @param signalA first signal to compare
     * @param signalB second signal to compare
     * @param minAbsA minimum magnitude for {@code signalA} before comparison is enforced
     * @param minAbsB minimum magnitude for {@code signalB} before comparison is enforced
     * @return {@code true} when either signal is below threshold or signs match; otherwise {@code false}
     */
    private boolean signsMatchWithThreshold(double signalA, double signalB, double minAbsA, double minAbsB) {
        if (Math.abs(signalA) < minAbsA || Math.abs(signalB) < minAbsB) {
            return true;
        }
        return Math.signum(signalA) == Math.signum(signalB);
    }

    /**
     * Computes the absolute-encoder offset that maps a measured raw angle to a desired module angle.
     *
     * @param measuredRawRad measured raw absolute angle in radians
     * @param desiredModuleAngleRad desired module angle in radians
     * @param reversed whether the absolute encoder reading is inverted
     * @return recommended offset in radians, wrapped to {@code [0, 2pi)}
     */
    private double calculateRecommendedOffsetRad(
        double measuredRawRad,
        double desiredModuleAngleRad,
        boolean reversed
    ) {
        double sign = reversed ? -1.0 : 1.0;
        return MathUtil.inputModulus(measuredRawRad - (desiredModuleAngleRad * sign), 0.0, TWO_PI);
    }

    /**
     * Returns the circular mean of sampled raw absolute angles for a module.
     *
     * @param moduleIndex module index in standard order (front-left, front-right, back-left, back-right)
     * @return average raw angle in radians wrapped to {@code [0, 2pi)}
     */
    private double getAverageRawAngleRad(int moduleIndex) {
        return MathUtil.inputModulus(
            Math.atan2(calibrationRawSinSum[moduleIndex], calibrationRawCosSum[moduleIndex]),
            0.0,
            TWO_PI
        );
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
