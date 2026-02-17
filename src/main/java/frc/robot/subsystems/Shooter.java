package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final VictorSP shooterMotor;
    private final double shooterSpeed;

    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    public Shooter(double shooterSpeed){
        this.shooterMotor = new VictorSP(5);
        this.shooterSpeed = shooterSpeed;
    }

    public void run(){
        if(m_bottomlimitSwitch.get() || m_toplimitSwitch.get()){
            shooterMotor.set(0);
        } else {
            shooterMotor.set(shooterSpeed);
        }
        
    }

    public void stop(){
        shooterMotor.set(0);
    }

}
