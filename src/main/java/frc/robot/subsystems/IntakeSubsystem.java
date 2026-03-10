package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final PWMSparkMax intakeMotor;
    private boolean intakeEnabled = false;

    public IntakeSubsystem(){
        intakeMotor = new PWMSparkMax(IntakeConstants.intakeMotorPort);
        intakeMotor.setInverted(IntakeConstants.intakeMotorReversed);
    }

    public void startIntakeMotor(double speed){
        intakeEnabled = true;
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor(){
        intakeEnabled = false;
        intakeMotor.stopMotor();
    }

    public void toggleIntakeMotor() {
        if (intakeEnabled) {
            stopIntakeMotor();
            return;
        }

        startIntakeMotor(IntakeConstants.intakeFullSpeed);
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }
}
