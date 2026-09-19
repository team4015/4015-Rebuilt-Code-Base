package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public Command runShooter2(){
        return startEnd(
                () -> {
                    shooterMotor.set(shooterSpeed);
                    System.out.println("Shooter Command Called");
                    SmartDashboard.putBoolean("shooterRunning?", true);
                },
                () -> {
                    shooterMotor.set(0);
                    System.out.println("Shooter Command Stopped");
                    SmartDashboard.putBoolean("shooterRunning?", false);
                }
        );
    }

    public Command runIndexer2(){
        return startEnd(
                () -> {
                    indexerMotor.set(indexerSpeed);
                    System.out.println("Indexer Command Called");
                    SmartDashboard.putBoolean("indexerRunning?", true);
                },
                () -> {
                    indexerMotor.set(0);
                    System.out.println("Indexer Command Stopped");
                    SmartDashboard.putBoolean("indexerRunning?", false);
                }
        );
    }

    
    //Stop everything
    public void stop(){
        shooterMotor.set(0);
        indexerMotor.set(0);
    }
}
