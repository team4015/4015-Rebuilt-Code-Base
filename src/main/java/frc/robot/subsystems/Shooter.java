package frc.robot.subsystems;


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
