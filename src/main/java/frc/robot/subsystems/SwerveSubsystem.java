package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Meter;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    double maximumSpeed = 5;


    SwerveDrive swerveDrive;
    double deadband = Constants.OperatorConstants.DEADBAND;
    private final CommandPS4Controller driverCtrl = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

    
    public SwerveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, new Pose2d(new Translation2d(Meter.of(5),Meter.of(5)), Rotation2d.fromDegrees(90)));

        } catch (IOException e) {
            throw new RuntimeException("Swerve Drive directory not found. Finish those config files!");
        }

        swerveDrive.zeroGyro();
        swerveDrive.getPose();
        
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command. NOT BEING USED */
     

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    private boolean fieldOriented = true;
    
    
        
        
    
        public void toggleFieldOriented(){
            fieldOriented = !fieldOriented;
        }
    
        public boolean isFieldOriented(){
            return fieldOriented;
        }
    
    
    
         public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
      {
        return run(() -> {
          // Make the robot move
          swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                            true,
                            false);
        });
      }
        
        
        /*public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                DoubleSupplier angularRotationX) {
                
            System.out.println("Reached driveCommand");
    
            return run(() -> {
                double maxVelocity = swerveDrive.getMaximumChassisVelocity();
                
                //Getting the double from the double suppliers and multiplying them by their max velocity
                double vxMetersPerSecond = MathUtil.applyDeadband(translationX.getAsDouble(), deadband);
                double vyMetersPerSecond = MathUtil.applyDeadband(translationY.getAsDouble(), deadband);
                double angularRotation = MathUtil.applyDeadband(angularRotationX.getAsDouble(), deadband);
                
                vxMetersPerSecond = vxMetersPerSecond * maxVelocity;
                vyMetersPerSecond = vyMetersPerSecond * maxVelocity;
                angularRotation = angularRotation * swerveDrive.getMaximumChassisAngularVelocity();
    
                Translation2d translation = new Translation2d(vxMetersPerSecond,vyMetersPerSecond);
                Translation2d scaledTranslation = SwerveMath.scaleTranslation(translation);
                
                swerveDrive.drive(translation, angularRotation, true, true);
        
                System.out.println("In Run Method" + "\nvxMetersPerSecond:" + vxMetersPerSecond + " \nvyMetersPerSecond: " + vyMetersPerSecond);
                System.out.println(" ");
    
                SmartDashboard.putNumber("horizontalMovement", vxMetersPerSecond);
                SmartDashboard.putNumber("verticalMovement", vyMetersPerSecond);
                SmartDashboard.putNumber("angularRotation", angularRotation);
    
                SmartDashboard.putBoolean("button1Pressed", driverCtrl.getHID().getRawButton(1));
                SmartDashboard.putBoolean("button2Pressed", driverCtrl.getHID().getRawButton(2));
                SmartDashboard.putBoolean("button3Pressed", driverCtrl.getHID().getRawButton(3));
                SmartDashboard.putBoolean("button4Pressed", driverCtrl.getHID().getRawButton(4));
    
                SmartDashboard.putNumber("leftXJoystick", driverCtrl.getLeftX());
                SmartDashboard.putNumber("leftYJoystick", driverCtrl.getLeftY());
                SmartDashboard.putNumber("rightXJoystick", driverCtrl.getRightX());
            });
    
    
        }*/
    
    
    
        public double round(double value, int places) {
            double scale = Math.pow(10, places);
            return Math.round(value * scale) / scale;
        }


        public SwerveDrive getSwerveDrive(){
            return swerveDrive;
        }
    
        public void driveFieldOriented(ChassisSpeeds velocity)
      {
        swerveDrive.driveFieldOriented(velocity);
      }
    

      private NetworkTableInstance inst = NetworkTableInstance.getDefault();
      private NetworkTable table = inst.getTable("SmartDashboard/swerve");
      DoubleArraySubscriber measuredChassisSpeedsSub = table.getDoubleArrayTopic("measuredChassisSpeeds").subscribe(new double[] {});

      public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
      {
        return run(
            () -> {
              swerveDrive.driveFieldOriented(velocity.get());
              SmartDashboard.putBoolean("button1Pressed", driverCtrl.getHID().getSquareButton());
                SmartDashboard.putBoolean("button2Pressed", driverCtrl.getHID().getCrossButton());
                SmartDashboard.putBoolean("button3Pressed", driverCtrl.getHID().getCircleButton());
                SmartDashboard.putBoolean("button4Pressed", driverCtrl.getHID().getTriangleButton());
    
                SmartDashboard.putString("leftXJoystick", String.format("%.2f", driverCtrl.getLeftX()));
                SmartDashboard.putString("leftYJoystick", String.format("%.2f", driverCtrl.getLeftY()));
                SmartDashboard.putString("rightXJoystick", String.format("%.2f", driverCtrl.getRightX()));
    
                double[] measuredChassisSpeeds = measuredChassisSpeedsSub.get();

                for(int i = 0; i < measuredChassisSpeeds.length; i++){
                    measuredChassisSpeeds[i] = Math.round(measuredChassisSpeeds[i] * 100.0) / 100.0;
                    if(i == 0){SmartDashboard.putNumber("HorizontalSpeedX", measuredChassisSpeeds[i]);} 
                    else if(i == 1){SmartDashboard.putNumber("VerticalSpeedY", measuredChassisSpeeds[i]);}
                    else {SmartDashboard.putNumber("rotSpeedTheta", measuredChassisSpeeds[i]);}

                    
                }


                SmartDashboard.putNumberArray("RoundedMeasuredChassisSpeeds", measuredChassisSpeeds);

        });


            
  }
    

}