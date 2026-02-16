package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final VictorSP intakeMotor;
    private final double intakeSpeed;

    public Intake(){
        intakeMotor = new VictorSP(4);
        intakeSpeed = 0.70;
    }

    public void setSpeed(double motorSpeed){
        intakeMotor.set(motorSpeed);
    }

    public void stop(){
        intakeMotor.set(0);
    }


}
