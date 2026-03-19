package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveSubsystem extends SubsystemBase {

    double maximumSpeed = 5;


    SwerveDrive swerveDrive;
    double deadband = Constants.OperatorConstants.DEADBAND;
    private final CommandXboxController driverCtrl = new CommandXboxController(OperatorConstants.kDriverControllerPort);



    public SwerveSubsystem(File directory) {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);

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
    
    
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
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


    }



    public SwerveDrive getSwerveDrive(){
        return swerveDrive;
    }

    

}