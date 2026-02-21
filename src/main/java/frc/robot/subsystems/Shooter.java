package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    //Declare motors, speeds, default hood direction, and limit switches
    
    private final SparkMax shooterMotor;
    private final SparkMax hoodMotor;
    private final SparkMax indexerMotor;

    private final double shooterSpeed;
    private final double hoodSpeed;
    private final double indexerSpeed;

    private boolean hoodGoDown = true;
    
    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    //Initialize the shooter

    public Shooter(double shooterSpeed, double hoodSpeed, double indexerSpeed){
        this.shooterMotor = new SparkMax(12, MotorType.kBrushless);
        this.hoodMotor = new SparkMax(13, MotorType.kBrushless);
        this.indexerMotor = new SparkMax(14, MotorType.kBrushless);

        this.shooterSpeed = shooterSpeed;
        this.hoodSpeed = hoodSpeed;
        this.indexerSpeed = indexerSpeed;
    }

    //Run the shooter

    public void runShooter(){
        shooterMotor.set(shooterSpeed);
    }

    // Extend or retract the shooter hood

    public void runHood(){
        if(m_bottomlimitSwitch.get()){
            hoodGoDown = false;
            hoodMotor.set(0);    
            return;

        } else if(m_toplimitSwitch.get()){
            hoodGoDown = true;
            hoodMotor.set(0);
            return; 
            
        } else {
            hoodMotor.set((hoodGoDown) ? hoodSpeed : -hoodSpeed);
        }

    }

    //Run the indexer

    public void runIndexer(){
        indexerMotor.set(indexerSpeed);
    }
    
    //Stop everything

    public void stop(){
        shooterMotor.set(0);
        hoodMotor.set(0);
        indexerMotor.set(0);
    }

}
