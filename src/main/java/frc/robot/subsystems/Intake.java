package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final VictorSP intakeMotor;
    private final double intakeSpeed;

    public Intake(double intakeSpeed){
        this.intakeMotor = new VictorSP(4);
        this.intakeSpeed = intakeSpeed;
    }

    public void run(){
        intakeMotor.set(intakeSpeed);
    }

    public void stop(){
        intakeMotor.set(0);
    }


}
