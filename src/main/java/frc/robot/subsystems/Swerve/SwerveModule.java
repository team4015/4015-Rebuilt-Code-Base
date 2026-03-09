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

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return normalizeTurningAngle(turningEncoder.getPosition());
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double correctedAngle = getRawAbsoluteEncoderRad() - absoluteEncoderOffsetRad;
        if (absoluteEncoderReversed) {
            correctedAngle = -correctedAngle;
        }
        return normalizeTurningAngle(correctedAngle);
    }

    public double getRawAbsoluteEncoderRad() {
        double ratio = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
        ratio = MathUtil.clamp(ratio, 0.0, 1.0);
        return normalizeAngle0To2Pi(ratio * TWO_PI);
    }

    public double getRawAbsoluteEncoderDeg() {
        return Math.toDegrees(getRawAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderDeg() {
        return Math.toDegrees(getAbsoluteEncoderRad());
    }

    public double normalizeAngle0To2Pi(double angleRad) {
        return MathUtil.inputModulus(angleRad, 0.0, TWO_PI);
    }

    public double normalizeTurningAngle(double angleRad) {
        return MathUtil.angleModulus(angleRad);
    }

    public double getAbsoluteEncoderOffsetRad() {
        return absoluteEncoderOffsetRad;
    }

    public boolean isAbsoluteEncoderReversed() {
        return absoluteEncoderReversed;
    }

    public int getAbsoluteEncoderChannel() {
        return absoluteEncoder.getChannel();
    }

    public double getDriveAppliedOutput() {
        return driveMotor.get();
    }

    public double getTurningAppliedOutput() {
        return turningMotor.get();
    }

    public boolean isDriveMotorReversed() {
        return driveMotorReversed;
    }

    public boolean isTurningMotorReversed() {
        return turningMotorReversed;
    }

    public void resetEncoder() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

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

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setBrakeMode(boolean enabled) {
        IdleMode idleMode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        driveConfig.idleMode(idleMode);
        turningConfig.idleMode(idleMode);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Rotation2d getTurningRotation() {
        return Rotation2d.fromRadians(getTurningPosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningRotation());
    }
}
