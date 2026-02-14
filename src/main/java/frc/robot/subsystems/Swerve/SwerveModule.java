/*************************************************************************************************
 @Name: Gursahaj Chawla
 @Date: 2/10/2026
 @File: SwerveModule.java
 @Description: This class in the program and the setup for an individual swerve modules in which
 four objects of this class will be created inorder to combine them through IK motion
 ***********************************************************************************************/

package frc.robot.subsystems.Swerve;

//WPLib library imports and other external library imports
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    //SparKMax motor controller for single drive and turning motor
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    //variables to configure the motors using the SparkMaxConfig class for a single drive and turning motor
    private SparkMaxConfig driveConfig;
    private SparkMaxConfig turningConfig;

    //Variables for the Relative Encoder for a single drive and turning encoder
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    //PI controller for thr turning (accurate turning for smooth accurate turning for each module)
    private final PIDController turningPIDcontroller;

    //Analog input for the absolute encoder for a single swerve module
    private final AnalogInput absoluteEncoder;
    //Boolean variables to see if the absolute encoder is reversed
    private final boolean absoluteEncoderReversed;
    //variable to get the absolute encoder offset in radians
    private final double absoluteEncoderOffsetRad;

    //The constructor of the class which has parameters of most of the final variables above
    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
        int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
            
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset; //initialize variable
        this.absoluteEncoderReversed = absoluteEncoderReversed; //initialize variable
        absoluteEncoder = new AnalogInput(absoluteEncoderID); //initialize the absolute Encoder by inputting the ID

        //initialize the drive and turning motor by giving the motor ID and the type of motor (brushless motor)
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        //initialize the drive and turn motor configuration variables
        driveConfig = new SparkMaxConfig();
        turningConfig = new SparkMaxConfig();

        //configure the variable to set the idle mode and see if its inverted the drive motor
        driveConfig
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kBrake);

        //set up the drive encoder values for the position and velocity conversion factor
        //This will also see ht measure period (how often it will read) and the average depth (smooth the data)
        driveConfig.encoder //change value
            .positionConversionFactor(ModuleConstants.driveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.driveEncoderRPM2MeterPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        //configure the variable to set the idle mode and see if its inverted the turn motor
        turningConfig
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kBrake);

        //set up the turn encoder values for the position and velocity conversion factor
        //This will also see ht measure period (how often it will read) and the average depth (smooth the data)
        turningConfig.encoder //change value
            .positionConversionFactor(ModuleConstants.turningEncoderRot2Rad)
            .velocityConversionFactor(ModuleConstants.turningEncoderRPM2RadPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        //Use all the configuration values set for the drive and turning motor and configure then directly to the motor
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Get encoder values from the Drive Encoder
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //set the PID controller by only changes the proportional variable not the derivative or integral constants
        turningPIDcontroller = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDcontroller.enableContinuousInput(-Math.PI, Math.PI); //set it such that it will accept values from -π to π

        resetEncoder(); //reset the encoders
    }

    //get method for the drive motor position
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    //get method for the turn motor position
    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    //get the drive motor velocity
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    //get the turn motor velocity
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    //get the absolute encoder values in radius
    public double getAbsoluteEncoderRad(){
        //gets the voltages from the encoder then get the actual rail voltage which is divided to get the angle
        double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
        angle *= 2.0 * Math.PI; //convert this angle into radians
        angle -= absoluteEncoderOffsetRad;  //subtract the offset values to get its relative 0 to other modules
        return angle * (absoluteEncoderReversed ? -1.0: 1.0); //result the angles and multiples by 1 or -1 if the absolute encoder is reversed
    }

    //this method set the position to 0 and set the turning encoder to the absolute encoder in radians
    public void resetEncoder(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    //This class to combine two separate operations inorder one swerve module (SwerveModuleState)
    //Typically the wheels speed and wheel radians
    public SwerveModuleState getState(){
        //the getDriveVelocity which returns how fast when is spinning linearly.
        //Used Rotational to normalize angles and makes it rotate, it inputs the turningPosition which is the steering angle
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    //This method that takes the target wheel and angle and find the most efficient way to reach that angle
    //The commands are the drive motor telling how fast to roll and the turning motor (where to point)
    //Get the SwerveModuleState state to see the angle and rotation of the swerve module
    public void setDesiredState(SwerveModuleState state){
        //This deadband won't twitch when stopped to remove the jitter, wheel hunting and early exits
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        //this will optimize the state of the motor to remove unnecessary wheel rotation (over rotating)
        //e.g instead of rotating 340 degrees it will just rotate  -20 degrees
        state = SwerveModuleState.optimize(state, getState().angle);
        //This will set the speed of the drive motor from -1.0 to 1.0
        //desired speed/max speed = motor output scalar value
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.physicalMaxSpeedMetersPerSecond);
        //This will set the speed of the turning motor frm the measurement of the current angle and gets a setpoint
        //Uses the PID to compute how hard it will to turn the motor and uses closed-loop contorl
        turningMotor.set(turningPIDcontroller.calculate(getTurningPosition(), state.angle.getRadians()));
        //This is used for debugging output
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
    //Helps to configure the motor for whether the brakes are required
    public void setBrakeMode(boolean enabled){
        //IdleMode refers to how the motor behaves when it is receiving a neutral command
        //If the brakes are enables, then it could brake, it the brakes are not enables, then it will be in a Coast state
        IdleMode idleMode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        //Configure the IdleMode of the robot with the statement result above
        driveConfig.idleMode(idleMode);
        turningConfig.idleMode(idleMode);
        //send the updated configuration from above to the motors
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //This gets the absolute steering encoder and converts the angle into Rotational2d WPILib angle and returns the current steering direction
    public Rotation2d getTurningRotation() {
        return Rotation2d.fromRadians(getAbsoluteEncoderRad());
    }

    //This will get the position of the driving and turning motor
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(), 
            getTurningRotation()
        );
    }
}