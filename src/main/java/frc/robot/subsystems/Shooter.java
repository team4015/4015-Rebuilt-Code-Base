package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemConstants.*;


public class Shooter extends SubsystemBase{

    //Declare motors, speeds, default hood direction, and limit switches
    private final PWMSparkMax shooterMotor;
    private final PWMSparkMax indexerMotor;

    private final double shooterSpeed;
    private final double indexerSpeed;

    //Initialize the shooter

    public Shooter(){
        this.shooterMotor = new PWMSparkMax(shooterMotorPort); //Constants File
        this.indexerMotor = new PWMSparkMax(indexerMotorPort); //Constants File
    
        this.shooterSpeed = ConstShooterSpeed; //Constants File
        this.indexerSpeed = ConstIndexerSpeed; //Constants File
    }

    public void runShooter(){
        shooterMotor.set(shooterSpeed);
    }

    public void runIndexer(){
        indexerMotor.set(indexerSpeed);
    }

    public void stopShooter(){
        shooterMotor.set(0);
    }

    public void stopIndexer(){
        indexerMotor.set(0);
    }
    
    //Stop everything
    public void stop(){
        shooterMotor.set(0);
        indexerMotor.set(0);
    }
}
