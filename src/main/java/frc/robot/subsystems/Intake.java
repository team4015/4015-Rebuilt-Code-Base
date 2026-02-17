package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final VictorSP intakeMotor;
    private final double intakeSpeed;

    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    public Intake(double intakeSpeed){
        this.intakeMotor = new VictorSP(4);
        this.intakeSpeed = intakeSpeed;
    }

    public void run(){
        if(m_bottomlimitSwitch.get() || m_toplimitSwitch.get()){
            intakeMotor.set(0);
        } else {
            intakeMotor.set(intakeSpeed);
        }
        
    }

    public void stop(){
        intakeMotor.set(0);
    }


}
