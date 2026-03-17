package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase{

    //Declare motors, speeds, default hood direction, and limit switches
    
    private final PWMSparkMax shooterMotor;
    private final PWMSparkMax indexerMotor;

    private final double shooterSpeed;
    private final double indexerSpeed;
    
  
    //Initialize the shooter

    public Shooter(double shooterSpeed, double indexerSpeed){
        this.shooterMotor = new PWMSparkMax(2);
        this.indexerMotor = new PWMSparkMax(0);
    
        this.shooterSpeed = shooterSpeed;
        this.indexerSpeed = indexerSpeed;
    }

    //Run the shooter

    public void runShooter(){
        shooterMotor.set(shooterSpeed);
    }

    // Extend or retract the shooter hood


    //Run the indexer

    public void runIndexer(){
        indexerMotor.set(indexerSpeed);
    }
    
    //Stop everything

    public void stop(){
        shooterMotor.set(0);
        indexerMotor.set(0);
    }

}
