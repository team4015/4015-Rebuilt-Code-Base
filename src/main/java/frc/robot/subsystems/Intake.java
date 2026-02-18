package frc.robot.subsystems;

import java.nio.channels.Channel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final VictorSP intakeMotor;
    private final VictorSP extendMotor;
    private final double intakeSpeed;
    private final double extendSpeed;
    private boolean extend = true;



    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    public Intake(double intakeSpeed, double extendSpeed){
        this.intakeMotor = new VictorSP(4);
        this.extendMotor = new VictorSP(6);
        this.intakeSpeed = intakeSpeed;
        this.extendSpeed = extendSpeed;
    }

    public void runIntake(){
        intakeMotor.set(intakeSpeed);
    }


    public void extendIntake(){

        if(m_bottomlimitSwitch.get()){
            extend = false;
            extendMotor.set(0);    
            return;

        } else if(m_toplimitSwitch.get()){
            extend = true;
            extendMotor.set(0);
            return;

        } else {
            extendMotor.set((extend) ? extendSpeed : -extendSpeed);
        }

    }


    public void stop(){
        intakeMotor.set(0);
        extendMotor.set(0);
    }


}
