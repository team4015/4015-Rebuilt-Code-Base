/*************************************************************************************************
 @Name: Gursahaj Chawla
 @Date: 2/10/2026
 @File: Constants.java
 @Description: This static class has all the relevant constants and calculations for the Code Base
 Each static final class will have constants relevant for each subsystem. All constant values are
 stored in the JSON files via the deploy folder using the ConfigLoader class
 ***********************************************************************************************/
package frc.robot;

//WPILib imports and external library imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Config.ConfigLoader;
import frc.robot.Config.Drive.DriveConfig;
import frc.robot.Config.ExtendableHopperConfig;
import frc.robot.Config.IntakeConfig;

public final class Constants {
  //This is the DRIVE constant variables which has all the variables from the JSON file from the ConfigLoader
  public static final DriveConfig DRIVE = ConfigLoader.loadDriveConfig();
  public static final IntakeConfig INTAKE = ConfigLoader.loadIntakeConfig();
  public static final ExtendableHopperConfig EXTENDABLE_HOPPER = ConfigLoader.loadExtendableHopperConfig();

  //This static final class if for the ModuleConstants
  public static final class ModuleConstants {
    //Get the wheel diameter in meters converted from Inches --> Meters
    public static final double wheelDiameterMeters = Units.inchesToMeters(DRIVE.module.wheelDiameterInches);

    //Get the driveMotor and turnMotor Gear Ratio, the reciprocal operation is done to convert reduction ratio into overdrive ratio
    public static final double driveMotorGearRatio = 1.0 / DRIVE.module.driveGearRatio;
    public static final double turningMotorGearRatio = 1.0 / DRIVE.module.turningGearRatio;

    //This encoder rotation variables converts the Encoder position into meters
    //Calculation: This calculates the distance traveled in one full rotation by finding circumference and considering
    //the gear ratio
    public static final double driveEncoderRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;

    //Similar to the previous variable only difference is that the converts the angular displacement using
    // the full circumference of a circle 2π in radian and considering the gear ratio
    public static final double turningEncoderRot2Rad = turningMotorGearRatio * 2.0 * Math.PI;

    //Converting the drive and turning encoder into RPM by dividing the distance (driving encoder)
    //or the (turning encoder) by 60 seconds (1 minute)
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter / 60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad / 60.0;

    //This variable get the PID parameter for turning (proportional gain for turning)
    public static final double kPTurning = DRIVE.module.turningKP; 
  }

  public static final class DriveConstants {
    //This gets the track width (the lateral distance between the centerlines of the two wheels on the same axles) converted from inches --> meters
    public static final double trackWidth = Units.inchesToMeters(DRIVE.dimensions.trackWidthInches);

    //This gets the wheelbase (the horizontal distance between the centerlines of the two wheels on the front and rear wheels) converted from inches --> meters
    public static final double wheelBase = Units.inchesToMeters(DRIVE.dimensions.wheelBaseInches);

    //This variable uses the SwerveDriveKinematic class to do the important calculations
    //This converts the chassis speed vx, vy, yz into individual wheel speed and angles for each swerve module
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            //Translation2d represents the positions of the wheels relative to the robot center
            //new Translations2d(x, y), +X --> Forward, +Y --> Left, -X --> Backwards, -Y --> Right
            new Translation2d( wheelBase / 2, -trackWidth / 2), //front right wheel, +X, -Y
            new Translation2d( wheelBase / 2,  trackWidth / 2), //front left wheel, +X, +Y
            new Translation2d(-wheelBase / 2, -trackWidth / 2), //back right wheel, -X, -Y
            new Translation2d(-wheelBase / 2,  trackWidth / 2) // back right wheel, -X, +Y
        );

    //Get the motor ports for all the four swerve modules drive motors
    public static final int frontLeftDriveMotorPort = DRIVE.motors.drive.frontLeft;
    public static final int frontRightDriveMotorPort = DRIVE.motors.drive.frontRight;
    public static final int backLeftDriveMotorPort = DRIVE.motors.drive.backLeft;
    public static final int backRightDriveMotorPort = DRIVE.motors.drive.backRight;

    //Get the motor ports for all the four swerve modules turn motors
    public static final int frontLeftTurningMotorPort = DRIVE.motors.turning.frontLeft;
    public static final int frontRightTurningMotorPort = DRIVE.motors.turning.frontRight;
    public static final int backLeftTurningMotorPort = DRIVE.motors.turning.backLeft;
    public static final int backRightTurningMotorPort = DRIVE.motors.turning.backRight;

    //Get the Absolute encoder ports from each of the four swerve modules
    public static final int frontLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontLeft;
    public static final int frontRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.frontRight;
    public static final int backLeftDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backLeft;
    public static final int backRightDriveAbsoluteEncoderPort = DRIVE.encoders.absolutePorts.backRight;

    //Get the Encoder offset in radians (This makes is such that all of then understand relative 0
    public static final double frontLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontLeft;
    public static final double frontRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.frontRight;
    public static final double backLeftDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backLeft;
    public static final double backRightDriveAbsoluteEncoderOffsetRad = DRIVE.encoders.absoluteOffsetsRad.backRight;

    //Get the boolean states for each of the encoder values for if the drive encoders values are reversed
    public static final boolean frontLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.frontLeft;
    public static final boolean frontRightDriveEncoderReversed = DRIVE.encoders.driveReversed.frontRight;
    public static final boolean backLeftDriveEncoderReversed = DRIVE.encoders.driveReversed.backLeft;
    public static final boolean backRightDriveEncoderReversed = DRIVE.encoders.driveReversed.backRight;

    //Get the boolean states for each of the encoder values for if the turning encoders values are reversed
    public static final boolean frontLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.frontLeft;
    public static final boolean frontRightTurningEncoderReversed = DRIVE.encoders.turningReversed.frontRight;
    public static final boolean backLeftTurningEncoderReversed = DRIVE.encoders.turningReversed.backLeft;
    public static final boolean backRightTurningEncoderReversed = DRIVE.encoders.turningReversed.backRight;

    //Get the boolean states for each of the encoder values for if the absolute encoders values are reversed
    public static final boolean frontLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontLeft;
    public static final boolean frontRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.frontRight;
    public static final boolean backLeftDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backLeft;
    public static final boolean backRightDriveAbsoluteEncoderReversed = DRIVE.encoders.absoluteReversed.backRight;

    // Speed limits
    //this gets the maximum Speed of the robot to avoid overburning
    public static final double physicalMaxSpeedMetersPerSecond = DRIVE.limits.maxSpeedMps;

    //this gets the maximum angles speed of the turning components of the robot to avoid overburning
    public static final double physicalMaxAngularSpeedRadiansPerSecond = DRIVE.limits.maxAngularSpeedRadPerSec;

    //Maximum Speed in the teleop period for driving speed
    public static final double teleDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMetersPerSecond * DRIVE.limits.teleopSpeedScale;

    //Maximum turning Speed in the teleop period for turning speed
    public static final double teleDriveMaxAngularSpeedRadiansPerSecond = physicalMaxAngularSpeedRadiansPerSecond * DRIVE.limits.teleopSpeedScale;
  }

  public static final class IntakeConstants {
      public static final double intakePValue = INTAKE.pid.intake.p;
      public static final double intakeIValue = INTAKE.pid.intake.i;
      public static final double intakeDValue = INTAKE.pid.intake.d;

      public static final int intakeMotorPort = INTAKE.port.intake.system;

      public static final double intakeGearRatio = 1.0 / INTAKE.gearRatio.intake;

  }

  public static final class ExtendableHopperConstants {
      public static final double extendableHopperPValue = EXTENDABLE_HOPPER.pid.extendableHopper.p;
      public static final double extendableHopperIValue = EXTENDABLE_HOPPER.pid.extendableHopper.i;
      public static final double extendableHopperDValue = EXTENDABLE_HOPPER.pid.extendableHopper.d;

      public static final double extendableHopperGearRatio = 1.0 / EXTENDABLE_HOPPER.gearRatio.extendableHopper;

      public static final int extendableHopperMotorPort = EXTENDABLE_HOPPER.port.extendableHopper.system;

      public static final int frontLimitSwitch = EXTENDABLE_HOPPER.port.frontSwitch.limitSwitch;
      public static final int backLimitSwitch = EXTENDABLE_HOPPER.port.backSwitch.limitSwitch;

      public static boolean reverseExtendableHopper = false;
      public static double extendableHopperSpeed = 0.0;
  }

  //These ar the input-output constants for the robot and the controller like the controller port
  //and various pins on the joystick
  public static final class OIConstants {
    public static final int driverControllerPort = 0;

    public static final int driverYAxis = 1;
    public static final int driverXAxis = 0;
    public static final int driverRotAxis = 4;
    public static final int driverFieldOrientedButtonIdx = 1;
    public static final int driveUnlockSwerveButtonIdx = 2;
    public static final int calibrateAbsoluteEncoderButtonIdx = 3;

    public static final int intakePort = 10;
    public static final int extendableHopperPort = 11;

    //This is deadband to avoid drift on the robot
    public static final double deadband = 0.05;
  }
}
