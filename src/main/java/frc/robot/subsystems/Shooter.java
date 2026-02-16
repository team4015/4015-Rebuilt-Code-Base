package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final VictorSP shooterMotor;
    private final double shooterSpeed;

    public Shooter(double shooterSpeed){
        this.shooterMotor = new VictorSP(5);
        this.shooterSpeed = shooterSpeed;
    }

    public void run(){
        shooterMotor.set(shooterSpeed);
    }

    public void stop(){
        shooterMotor.set(0);
    }

}
