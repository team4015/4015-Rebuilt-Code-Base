package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Config.ConfigLoader;
import frc.robot.Config.DriveConfig;
import frc.robot.Config.ExtendableHopperConfig;
import frc.robot.Config.IntakeConfig;
import frc.robot.Config.VisionConfig;

public final class Constants {
    public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
    public static final IntakeConfig INTAKE = ConfigLoader.loadIntakeConfig();
    public static final ExtendableHopperConfig EXTENDABLE_HOPPER = ConfigLoader.loadExtendableHopperConfig();
    public static final VisionConfig VISION = ConfigLoader.loadVisionConfig();

    private Constants() {
    }

    public static final class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);
        public static final double driveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;
        public static final double turningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;
        public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;
        public static final double turningEncoderRot2Rad = turningMotorGearRatio * 2.0 * Math.PI;
        public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60.0;
        public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad / 60.0;
        public static final double kPTurning = DRIVE.module.turningKP;

        private ModuleConstants() {
        }
    }

    public static final class DriveConstants {
        public static final double trackWidth = Units.inchesToMeters(DRIVE.dimensions.trackWidthInches);
        public static final double wheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

        public static final SwerveDriveKinematics driveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );

        public static final int frontLeftDriveMotorPort = DRIVE.motors.drive.frontLeft;
        public static final int frontRightDriveMotorPort = DRIVE.motors.drive.frontRight;
        public static final int backLeftDriveMotorPort = DRIVE.motors.drive.backLeft;
        public static final int backRightDriveMotorPort = DRIVE.motors.drive.backRight;

        public static final int frontLeftTurningMotorPort = DRIVE.motors.turning.frontLeft;
        public static final int frontRightTurningMotorPort = DRIVE.motors.turning.frontRight;
        public static final int backLeftTurningMotorPort = DRIVE.motors.turning.backLeft;
        public static final int backRightTurningMotorPort = DRIVE.motors.turning.backRight;

        public static final int frontLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontLeft;
        public static final int frontRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontRight;
        public static final int backLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backLeft;
        public static final int backRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backRight;

        public static final double frontLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontLeft;
        public static final double frontRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontRight;
        public static final double backLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backLeft;
        public static final double backRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backRight;

        public static final boolean frontLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.frontLeft;
        public static final boolean frontRightDriveEncoderReversed = DRIVE.encoders.driveReversed.frontRight;
        public static final boolean backLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.backLeft;
        public static final boolean backRightDriveEncoderReversed = DRIVE.encoders.driveReversed.backRight;

        public static final boolean frontLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.frontLeft;
        public static final boolean frontRightTurningEncoderReversed = DRIVE.encoders.turningReversed.frontRight;
        public static final boolean backLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.backLeft;
        public static final boolean backRightTurningEncoderReversed = DRIVE.encoders.turningReversed.backRight;

        public static final boolean frontLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontLeft;
        public static final boolean frontRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontRight;
        public static final boolean backLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backLeft;
        public static final boolean backRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backRight;

        public static final double physicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;
        public static final double physicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;
        public static final double teleDriveMaxSpeedMetersPerSecond =
            physicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;
        public static final double teleDriveMaxAngularSpeedRadiansPerSecond =
            physicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;

        private DriveConstants() {
        }
    }

    public static final class IntakeConstants {
        public static final double intakePValue = INTAKE.pid.intake.p;
        public static final double intakeIValue = INTAKE.pid.intake.i;
        public static final double intakeDValue = INTAKE.pid.intake.d;
        public static final int intakeMotorPort = INTAKE.port.intake.system;
        public static final double intakeGearRatio = 1.0 / INTAKE.gearRatio.intake;
        public static final boolean intakeMotorReversed = INTAKE.settings.intakeMotorReversed;
        public static final double intakeFullSpeed = INTAKE.settings.fullSpeed;

        private IntakeConstants() {
        }
    }

    public static final class ExtendableHopperConstants {
        public static final double extendableHopperPValue = EXTENDABLE_HOPPER.pid.extendableHopper.p;
        public static final double extendableHopperIValue = EXTENDABLE_HOPPER.pid.extendableHopper.i;
        public static final double extendableHopperDValue = EXTENDABLE_HOPPER.pid.extendableHopper.d;
        public static final double extendableHopperGearRatio = 1.0 / EXTENDABLE_HOPPER.gearRatio.extendableHopper;
        public static final int extendableHopperMotorPort = EXTENDABLE_HOPPER.port.extendableHopper.system;
        public static final int frontLimitSwitch = EXTENDABLE_HOPPER.port.frontSwitch.limitSwitch;
        public static final int backLimitSwitch = EXTENDABLE_HOPPER.port.backSwitch.limitSwitch;
        public static final double matchStartForwardSpeed = EXTENDABLE_HOPPER.settings.matchStartForwardSpeed;
        public static final double manualSpeed = EXTENDABLE_HOPPER.settings.manualSpeed;
        public static boolean reverseExtendableHopper = false;
        public static double extendableHopperSpeed = 0.0;

        private ExtendableHopperConstants() {
        }
    }

    public static final class OIConstants {
        public static final int driverControllerPort = DRIVE.oi.driverControllerPort;
        public static final int driverYAxis = DRIVE.oi.driverYAxis;
        public static final int driverXAxis = DRIVE.oi.driverXAxis;
        public static final int driverRotAxis = DRIVE.oi.driverRotAxis;
        public static final int intakeToggleButtonIdx = DRIVE.oi.intakeToggleButtonIdx;
        public static final int driverLeftTriggerAxis = DRIVE.oi.driverLeftTriggerAxis;
        public static final int driverRightTriggerAxis = DRIVE.oi.driverRightTriggerAxis;
        public static final int driverFieldOrientedButtonIdx = DRIVE.oi.driverFieldOrientedButtonIdx;
        public static final int driveUnlockSwerveButtonIdx = DRIVE.oi.driveUnlockSwerveButtonIdx;
        public static final int aimAtTagButtonIdx = DRIVE.oi.aimAtTagButtonIdx;
        public static final int calibrateAbsoluteEncoderButtonIdx = DRIVE.oi.calibrateAbsoluteEncoderButtonIdx;
        public static final int aprilTagAlignButtonIdx = DRIVE.oi.aprilTagAlignButtonIdx;
        public static final double triggerPressedThreshold = DRIVE.oi.triggerPressedThreshold;
        public static final int intakePort = DRIVE.oi.intakePort;
        public static final int extendableHopperPort = DRIVE.oi.extendableHopperPort;
        public static final double deadband = DRIVE.oi.deadband;

        private OIConstants() {
        }
    }

    public static final class LimelightConstants {
        public static final String limelightName = VISION.limelight.name;
        public static final double lensHeightMeters = Units.inchesToMeters(VISION.limelight.lensHeightInches);
        public static final double mountAngleDegrees = VISION.limelight.mountAngleDegrees;
        public static final double defaultTagHeightMeters = Units.inchesToMeters(VISION.limelight.defaultTagHeightInches);

        private LimelightConstants() {
        }
    }

    public static final class VisionConstants {
        public static final double aimKp = VISION.aim.kp;
        public static final double aimMaxAngularSpeedRadPerSec = VISION.aim.maxAngularSpeedRadPerSec;
        public static final double aimToleranceDegrees = VISION.aim.toleranceDegrees;

        private VisionConstants() {
        }
    }
}
