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
 * Represents a single swerve module consisting of a drive motor, turning motor, and absolute encoder.
 */
public class SwerveModule {
    private static final double TWO_PI = 2.0 * Math.PI;

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private SparkMaxConfig driveConfig;
    private SparkMaxConfig turningConfig;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final PIDController turningPIDcontroller;
    private final AnalogInput absoluteEncoder;
    private final boolean driveMotorReversed;
    private final boolean turningMotorReversed;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /**
     * Creates and configures a swerve module.
     *
     * @param driveMotorID CAN ID of the drive motor
     * @param turningMotorID CAN ID of the turning motor
     * @param driveMotorReversed whether the drive motor output is inverted
     * @param turningMotorReversed whether the turning motor output is inverted
     * @param absoluteEncoderID analog input channel of the absolute encoder
     * @param absoluteEncoderOffset configured absolute encoder offset in radians
     * @param absoluteEncoderReversed whether the absolute encoder direction is inverted
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
            .idleMode(IdleMode.kBrake)
            // Limit drive current to match 40A breaker and reduce brownout risk.
            .smartCurrentLimit(40);

        driveConfig.encoder
            .positionConversionFactor(ModuleConstants.driveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.driveEncoderRPM2MeterPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        turningConfig
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kBrake)
            // Limit turning motor current to 40A for hardware protection.
            .smartCurrentLimit(40);

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
     * Returns drive wheel position in meters.
     *
     * @return drive distance in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns module steering angle in radians.
     *
     * @return steering angle in radians
     */
    public double getTurningPosition() {
        return normalizeTurningAngle(turningEncoder.getPosition());
    }

    /**
     * Returns drive velocity in meters per second.
     *
     * @return drive velocity in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns steering velocity in radians per second.
     *
     * @return steering velocity in radians per second
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Returns corrected absolute encoder angle in radians.
     *
     * @return corrected absolute angle in radians
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() /RobotController.getVoltage5V();
        angle *=2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    /**
     * Normalizes an angle into the continuous turning-controller range {@code [-pi, pi)}.
     *
     * @param angleRad input angle in radians
     * @return normalized angle in radians
     */
    public double normalizeTurningAngle(double angleRad) {
        return MathUtil.angleModulus(angleRad);
    }

    /** Resets the drive encoder and aligns the turning encoder to the absolute encoder. */
    public void resetEncoder() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Returns the current measured module state.
     *
     * @return current module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Commands the module to the desired state.
     *
     * @param state desired module state
     */
    public void setDesiredState(SwerveModuleState state) {
        state.optimize(getState().angle);
        double speed = state.speedMetersPerSecond;
        if (Math.abs(speed) < 0.05) {
            driveMotor.set(0);
            turningMotor.set(turningPIDcontroller.calculate(getTurningPosition(), state.angle.getRadians()));
            return;
        }

        driveMotor.set(speed / Constants.DriveConstants.physicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDcontroller.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    /** Stops both motors in the module. */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /**
     * Sets brake or coast mode on both module motors.
     *
     * @param enabled {@code true} for brake mode, {@code false} for coast mode
     */
    public void setBrakeMode(boolean enabled) {
        IdleMode idleMode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        driveConfig.idleMode(idleMode);
        turningConfig.idleMode(idleMode);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Returns the module steering angle as a {@link Rotation2d}.
     *
     * @return steering rotation
     */
    public Rotation2d getTurningRotation() {
        return Rotation2d.fromRadians(getTurningPosition());
    }

    /**
     * Returns the module position for odometry updates.
     *
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningRotation());
    }
}
