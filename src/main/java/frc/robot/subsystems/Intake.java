package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    //Declare motors, speeds, default intake extension direction, and limit switches
    private final SparkMax intakeMotor;
    private final SparkMax extendMotor;

    private final double intakeSpeed;
    private final double extendSpeed;

    private boolean extend = true;

    

    //Initialize the intake

    public Intake(double intakeSpeed, double extendSpeed){
        this.intakeMotor = new SparkMax(10,MotorType.kBrushless);
        this.extendMotor = new SparkMax(11, MotorType.kBrushless);

        this.intakeSpeed = intakeSpeed;
        this.extendSpeed = extendSpeed;
    }

    //Run the intake

    public void runIntake(){
        intakeMotor.set(intakeSpeed);
    }

    //Extend or retract the intake

    public void extendIntake(){
         
        extendMotor.set((extend) ? extendSpeed : -extendSpeed);
        
    }

    //Stop everything

    public void stop(){
        intakeMotor.set(0);
        extendMotor.set(0);
    }


}
