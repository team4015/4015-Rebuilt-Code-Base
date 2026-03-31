package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

import static frc.robot.Constants.SubsystemConstants.*;

public class Intake extends SubsystemBase{

    //Declare motors, speeds, default intake extension direction, and limit switches
    private final VictorSP intakeMotor;
    private final VictorSP extendMotor;

    private final double intakeSpeed;
    private final double extendSpeed;

    DigitalInput m_toplimitSwitch = new DigitalInput(0);
    DigitalInput m_bottomlimitSwitch = new DigitalInput(1);

    //Initialize the intake

    public Intake(){
        this.intakeMotor = new VictorSP(intakeMotorPort); //Constants File
        this.extendMotor = new VictorSP(extendIntakeMotorPort); //Constants File

        this.intakeSpeed = ConstIntakeSpeed; //Constants File
        this.extendSpeed = ConstExtendIntakeSpeed; //Constants File
    }

    //Run the intake
    public void runIntake(){intakeMotor.set(intakeSpeed);}
    //Outtake
    public void runOuttake(){intakeMotor.set(-intakeSpeed);}


    //Extend the intake
    public void extendIntake(){extendMotor.set(extendSpeed);}
    //Retract the intake
    public void retractIntake(){extendMotor.set(-extendSpeed);}


    //Stop intake
    public void stopIntake(){intakeMotor.set(0);}
    //Stop extending
    public void stopExtend(){extendMotor.set(0);}


}
