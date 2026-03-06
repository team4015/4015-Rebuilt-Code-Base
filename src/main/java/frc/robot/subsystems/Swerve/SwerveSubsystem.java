package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Swerve drivetrain subsystem.
 *
 * <p>This class owns the four module objects, heading/odometry updates, and a dedicated
 * Shuffleboard workflow for calibrating absolute steering encoder offsets.</p>
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

    // Running sum of raw absolute angle samples (radians) for each module.
    private final double[] calibrationRawSumRad = new double[4];
    // Number of samples accumulated in calibrationRawSumRad.
    private int calibrationSampleCount = 0;

    // Shuffleboard control entries (operator input).
    private GenericEntry calCollectEntry;
    private GenericEntry calResetEntry;
    private GenericEntry calTargetDegEntry;
    private GenericEntry calSampleCountEntry;

    // Shuffleboard per-module telemetry entries.
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

    // NavX gyro source for robot heading.
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // Tracks robot pose from heading + module positions.
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

    /**
     * Constructs the subsystem, builds calibration UI, and zeroes heading after a short startup
     * delay so sensors can stabilize.
     */
    public SwerveSubsystem() {
        initializeCalibrationDashboard();
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    /**
     * Resets gyro heading to zero.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns heading in degrees wrapped near [-180, 180].
     *
     * @return current robot heading in degrees
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360.0);
    }

    /**
     * Returns heading as a {@link Rotation2d}, converting NavX sign to WPILib convention.
     *
     * @return current robot rotation
     */
    public Rotation2d getRotation2d() {
        // NavX yaw sign is opposite of WPILib's CCW-positive convention.
        return Rotation2d.fromDegrees(-getHeading());
    }

    /**
     * Returns the current pose estimate.
     *
     * @return robot pose in meters/radians
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Resets odometry to a provided pose.
     *
     * @param pose target pose after reset
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

    /**
     * Periodic loop:
     * 1) process calibration controls,
     * 2) update odometry,
     * 3) publish calibration telemetry.
     */
    @Override
    public void periodic() {
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
    }

    /**
     * Builds the "Encoder Calibration" Shuffleboard tab and all widgets.
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
            // Arrange module panels in a 2x2 grid for quick matching with robot corners.
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
     * Executes calibration control actions from Shuffleboard.
     *
     * <p>Reset is treated as a one-shot action and forced back to false after execution.</p>
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

    /**
     * Clears all accumulated calibration sample data.
     */
    private void resetCalibrationSamples() {
        calibrationSampleCount = 0;
        for (int i = 0; i < calibrationRawSumRad.length; i++) {
            calibrationRawSumRad[i] = 0.0;
        }
    }

    /**
     * Adds one raw absolute-angle sample from each module.
     *
     * <p>Offsets are computed from average raw angle to reduce noise effects.</p>
     */
    private void captureCalibrationSample() {
        for (int i = 0; i < modules.length; i++) {
            calibrationRawSumRad[i] += modules[i].getRawAbsoluteEncoderRad();
        }
        calibrationSampleCount++;
    }

    /**
     * Pushes calibration telemetry values to Shuffleboard.
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
                ? (calibrationRawSumRad[i] / calibrationSampleCount)
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
     * Checks whether two signals have matching sign while both are above minimum thresholds.
     *
     * <p>Returns true when either signal is below threshold, so the dashboard does not report false
     * mismatch when the module is stationary.</p>
     *
     * @param signalA first signal
     * @param signalB second signal
     * @param minAbsA minimum magnitude for signalA to be considered active
     * @param minAbsB minimum magnitude for signalB to be considered active
     * @return true if signs match or if inactive due to thresholds
     */
    private boolean signsMatchWithThreshold(double signalA, double signalB, double minAbsA, double minAbsB) {
        if (Math.abs(signalA) < minAbsA || Math.abs(signalB) < minAbsB) {
            return true;
        }
        return Math.signum(signalA) == Math.signum(signalB);
    }

    /**
     * Computes the recommended absolute offset for one module.
     *
     * <p>Model used in module code:
     * {@code corrected = sign * (raw - offset)}, where sign is {@code -1} if reversed and {@code +1}
     * otherwise. Solving for offset gives:
     * {@code offset = raw - desired * sign}.
     * Result is wrapped to {@code [0, 2pi)} for stable storage in config.</p>
     *
     * @param measuredRawRad averaged raw module angle in radians
     * @param desiredModuleAngleRad target mechanical angle in radians
     * @param reversed whether this module absolute angle is reversed
     * @return recommended offset in radians wrapped to {@code [0, 2pi)}
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
     * Stops all module motors.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets brake mode for all modules.
     *
     * @param enabled true for brake, false for coast
     */
    public void setBrakeMode(boolean enabled) {
        frontLeft.setBrakeMode(enabled);
        frontRight.setBrakeMode(enabled);
        backLeft.setBrakeMode(enabled);
        backRight.setBrakeMode(enabled);
    }

    /**
     * Applies desired module states after desaturating to physical max speed.
     *
     * @param desiredStates ordered as [frontLeft, frontRight, backLeft, backRight]
     */
    public void setModulesStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * @return front-left corrected absolute steering angle in radians
     */
    public double getFrontLeftAbsoluteEncoderRad() {
        return frontLeft.getAbsoluteEncoderRad();
    }

    /**
     * @return front-right corrected absolute steering angle in radians
     */
    public double getFrontRightAbsoluteEncoderRad() {
        return frontRight.getAbsoluteEncoderRad();
    }

    /**
     * @return back-right corrected absolute steering angle in radians
     */
    public double getBackRightAbsoluteEncoderRad() {
        return backRight.getAbsoluteEncoderRad();
    }

    /**
     * @return back-left corrected absolute steering angle in radians
     */
    public double getBackLeftAbsoluteEncoderRad() {
        return backLeft.getAbsoluteEncoderRad();
    }
}
