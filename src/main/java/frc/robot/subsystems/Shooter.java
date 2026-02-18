package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final VictorSP shooterMotor;
    private final VictorSP hoodMotor;
    private final double shooterSpeed;
    private final double hoodSpeed;
    private boolean hoodGoDown = true;
    

    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    public Shooter(double shooterSpeed, double hoodSpeed){
        this.shooterMotor = new VictorSP(5);
        this.hoodMotor = new VictorSP(7);
        this.shooterSpeed = shooterSpeed;
        this.hoodSpeed = hoodSpeed;
    }

    public void runShooter(){
        shooterMotor.set(shooterSpeed);
    }

    public void runHood(){
        if(m_bottomlimitSwitch.get()){
            hoodGoDown = false;
            hoodMotor.set(0);    
        } else {
            hoodMotor.set((hoodGoDown) ? hoodSpeed : -hoodSpeed);
        }

        if(m_toplimitSwitch.get()){
            hoodGoDown = true;
            hoodMotor.set(0);
        } else {
            hoodMotor.set((hoodGoDown) ? hoodSpeed : -hoodSpeed);
        }

    }
    

    public void stop(){
        shooterMotor.set(0);
    }

}
