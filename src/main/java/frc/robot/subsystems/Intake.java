package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    //Declare motors, speeds, default intake extension direction, and limit switches
    private final VictorSP intakeMotor;
    private final VictorSP extendMotor;

    private final double intakeSpeed;
    private final double extendSpeed;

    private boolean extend = true;

    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    //Initialize the intake

    public Intake(double intakeSpeed, double extendSpeed){
        this.intakeMotor = new VictorSP(3);
        this.extendMotor = new VictorSP(0);

        this.intakeSpeed = intakeSpeed;
        this.extendSpeed = extendSpeed;
    }

    //Run the intake

    public void runIntake(){
        intakeMotor.set(intakeSpeed);
    }

    //Extend or retract the intake

    public void extendIntake(){
         
        /*if(m_bottomlimitSwitch.get()){
            extend = false;
            extendMotor.set(0);    
            return;

        } else if(m_toplimitSwitch.get()){
            extend = true;
            extendMotor.set(0);
            return;

        } else {
            extendMotor.set((extend) ? extendSpeed : -extendSpeed);
        }*/

        extendMotor.set((extend) ? extendSpeed : -extendSpeed);
        
    }

    //Stop everything

    public void stopIntake(){
        intakeMotor.set(0);
    }

    public void stopExtend(){
        extendMotor.set(0);
    }







}
