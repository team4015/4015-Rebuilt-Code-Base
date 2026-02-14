package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final VictorSP shooterMotor;
    private final double shooterSpeed;

    public Shooter(){
        shooterMotor = new VictorSP(5);
        shooterSpeed = 0.8;
    }

    public void setSpeed(boolean pressed){
        if (pressed){shooterMotor.set(shooterSpeed);} 
        else {shooterMotor.set(0);}
    }

}
