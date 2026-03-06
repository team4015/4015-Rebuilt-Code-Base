package frc.robot.subsystems.Swerve;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/**
 * One swerve module: drive motor, turning motor, relative encoders, and absolute steering encoder.
 *
 * <p>The absolute encoder value is corrected with configured offset/reversal and normalized before
 * being used to initialize and compare against turning encoder position.</p>
 */
public class SwerveModule {
    private static final double TWO_PI = 2.0 * Math.PI;

    // Motor controllers for wheel drive and steering.
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    // Mutable configs kept so brake/coast mode can be changed at runtime.
    private SparkMaxConfig driveConfig;
    private SparkMaxConfig turningConfig;

    // Relative motor encoders from the Spark Max controllers.
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    // Steering closed-loop controller.
    private final PIDController turningPIDcontroller;

    // Absolute analog encoder used for steering reference.
    private final AnalogInput absoluteEncoder;
    private final boolean driveMotorReversed;
    private final boolean turningMotorReversed;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /**
     * Creates and configures one swerve module.
     *
     * @param driveMotorID CAN ID of drive Spark Max
     * @param turningMotorID CAN ID of turning Spark Max
     * @param driveMotorReversed drive motor inversion setting
     * @param turningMotorReversed turning motor inversion setting
     * @param absoluteEncoderID analog channel for absolute encoder
     * @param absoluteEncoderOffset configured absolute encoder offset in radians
     * @param absoluteEncoderReversed true if absolute angle direction is reversed
     */
    public SwerveModule(
        int driveMotorID,
        int turningMotorID,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        int absoluteEncoderID,
        double absoluteEncoderOffset,
        boolean absoluteEncoderReversed
    ) {
        this.driveMotorReversed = driveMotorReversed;
        this.turningMotorReversed = turningMotorReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        turningConfig = new SparkMaxConfig();

        driveConfig
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kBrake);

        driveConfig.encoder
            .positionConversionFactor(ModuleConstants.driveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.driveEncoderRPM2MeterPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        turningConfig
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kBrake);

        turningConfig.encoder
            .positionConversionFactor(ModuleConstants.turningEncoderRot2Rad)
            .velocityConversionFactor(ModuleConstants.turningEncoderRPM2RadPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningPIDcontroller = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDcontroller.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoder();
    }

    /**
     * @return drive wheel position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * @return turning position normalized to {@code [-pi, pi)}
     */
    public double getTurningPosition() {
        return normalizeTurningAngle(turningEncoder.getPosition());
    }

    /**
     * @return drive velocity in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * @return turning angular velocity in radians per second
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Returns corrected absolute steering angle.
     *
     * <p>Pipeline:
     * 1) read raw absolute angle,
     * 2) subtract configured offset,
     * 3) apply optional reversal,
     * 4) normalize to {@code [-pi, pi)}.</p>
     *
     * @return corrected absolute steering angle in radians
     */
    public double getAbsoluteEncoderRad() {
        double correctedAngle = getRawAbsoluteEncoderRad() - absoluteEncoderOffsetRad;
        if (absoluteEncoderReversed) {
            correctedAngle = -correctedAngle;
        }
        return normalizeTurningAngle(correctedAngle);
    }

    /**
     * Reads raw absolute steering angle from analog voltage.
     *
     * <p>Voltage ratio is clamped to {@code [0, 1]} and mapped to {@code [0, 2pi)}.</p>
     *
     * @return raw absolute angle in radians
     */
    public double getRawAbsoluteEncoderRad() {
        double ratio = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
        ratio = MathUtil.clamp(ratio, 0.0, 1.0);
        return normalizeAngle0To2Pi(ratio * TWO_PI);
    }

    /**
     * @return raw absolute angle in degrees
     */
    public double getRawAbsoluteEncoderDeg() {
        return Math.toDegrees(getRawAbsoluteEncoderRad());
    }

    /**
     * @return corrected absolute angle in degrees
     */
    public double getAbsoluteEncoderDeg() {
        return Math.toDegrees(getAbsoluteEncoderRad());
    }

    /**
     * Normalizes an angle to {@code [0, 2pi)}.
     *
     * @param angleRad angle in radians
     * @return wrapped angle in radians
     */
    public double normalizeAngle0To2Pi(double angleRad) {
        return MathUtil.inputModulus(angleRad, 0.0, TWO_PI);
    }

    /**
     * Normalizes an angle to {@code [-pi, pi)} for steering control.
     *
     * @param angleRad angle in radians
     * @return wrapped angle in radians
     */
    public double normalizeTurningAngle(double angleRad) {
        return MathUtil.angleModulus(angleRad);
    }

    /**
     * @return configured absolute offset in radians
     */
    public double getAbsoluteEncoderOffsetRad() {
        return absoluteEncoderOffsetRad;
    }

    /**
     * @return whether absolute angle direction is reversed
     */
    public boolean isAbsoluteEncoderReversed() {
        return absoluteEncoderReversed;
    }

    /**
     * @return analog input channel used by the absolute encoder
     */
    public int getAbsoluteEncoderChannel() {
        return absoluteEncoder.getChannel();
    }

    /**
     * Returns the currently applied drive motor output command in range [-1, 1].
     *
     * @return drive motor command
     */
    public double getDriveAppliedOutput() {
        return driveMotor.get();
    }

    /**
     * Returns the currently applied turning motor output command in range [-1, 1].
     *
     * @return turning motor command
     */
    public double getTurningAppliedOutput() {
        return turningMotor.get();
    }

    /**
     * @return configured drive motor inversion flag from config
     */
    public boolean isDriveMotorReversed() {
        return driveMotorReversed;
    }

    /**
     * @return configured turning motor inversion flag from config
     */
    public boolean isTurningMotorReversed() {
        return turningMotorReversed;
    }

    /**
     * Resets encoders:
     * drive position to zero, and turning position to corrected absolute angle.
     */
    public void resetEncoder() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * @return current module state (speed + steering angle)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Applies a desired module state.
     *
     * <p>Behavior:
     * 1) deadband near zero speed to reduce jitter,
     * 2) optimize requested state to minimize wheel rotation,
     * 3) apply drive open-loop output and turning PID output.</p>
     *
     * @param state requested module state
     */
    public void setDesiredState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        if (Math.abs(speed) < 0.05) {
            driveMotor.set(0);
            turningMotor.set(turningPIDcontroller.calculate(getTurningPosition(), getTurningPosition()));
            return;
        }

        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.physicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDcontroller.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    /**
     * Stops both drive and turning motors.
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /**
     * Sets module neutral mode.
     *
     * @param enabled true for brake mode, false for coast mode
     */
    public void setBrakeMode(boolean enabled) {
        IdleMode idleMode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        driveConfig.idleMode(idleMode);
        turningConfig.idleMode(idleMode);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * @return turning angle as {@link Rotation2d}
     */
    public Rotation2d getTurningRotation() {
        return Rotation2d.fromRadians(getTurningPosition());
    }

    /**
     * @return module position for odometry integration
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningRotation());
    }
}
