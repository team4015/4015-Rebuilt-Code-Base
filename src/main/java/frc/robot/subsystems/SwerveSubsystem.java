package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Meter;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {

    double maximumSpeed = 5;

    SwerveDrive swerveDrive;
    double deadband = Constants.OperatorConstants.DEADBAND;
    RobotConfig config;
    private final CommandPS4Controller driverCtrl = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

    public SwerveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, new Pose2d(new Translation2d(Meter.of(5),Meter.of(5)), Rotation2d.fromDegrees(90)));
        //yeah
        } catch (IOException e) {
            throw new RuntimeException("Swerve Drive directory not found. Finish those config files!");
        }

  

        //for pathplanner reference
        swerveDrive.zeroGyro();
        swerveDrive.getPose();
        swerveDrive.getRobotVelocity();

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        AutoBuilder.configure(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
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


    public SwerveDrive getSwerveDrive(){
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity){
        swerveDrive.driveFieldOriented(velocity);
    }

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("SmartDashboard/swerve");
    DoubleArraySubscriber measuredChassisSpeedsSub = table.getDoubleArrayTopic("measuredChassisSpeeds").subscribe(new double[] {});
    double[] measuredChassisSpeeds = measuredChassisSpeedsSub.get();

    /*public Command drive(ChassisSpeeds velocity){
        return run(() -> {swerveDrive.drive(velocity);});
    }*/

    public void drive(ChassisSpeeds velocity){
        swerveDrive.drive(velocity);
        System.out.println("DRIVING: " + velocity);

    }

    public Command drive(Supplier<ChassisSpeeds> velocity){
        return run(
            () -> {
                swerveDrive.drive(velocity.get());
                System.out.println("DRIVING: " + velocity.get());

                SmartDashboard.putString("Pose2D", swerveDrive.getPose().toString());
                SmartDashboard.putString("leftXJoystick", String.format("%.2f", driverCtrl.getLeftX()));
                SmartDashboard.putString("leftYJoystick", String.format("%.2f", driverCtrl.getLeftY()));
                SmartDashboard.putString("rightXJoystick", String.format("%.2f", driverCtrl.getRightX()));


                for(int i = 0; i < measuredChassisSpeeds.length; i++){
                    measuredChassisSpeeds[i] = Math.round(measuredChassisSpeeds[i] * 100.0) / 100.0;

                    if(i == 0){SmartDashboard.putNumber("HorizontalSpeedX", measuredChassisSpeeds[i]);} 
                    else if(i == 1){SmartDashboard.putNumber("VerticalSpeedY", measuredChassisSpeeds[i]);}
                    else {SmartDashboard.putNumber("rotSpeedTheta", measuredChassisSpeeds[i]);}
                }

                SmartDashboard.putNumberArray("RoundedMeasuredChassisSpeeds", measuredChassisSpeeds);

            }
        );
    }
    
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
        return run(
            () -> {
                swerveDrive.driveFieldOriented(velocity.get());

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