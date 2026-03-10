package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Simple PWM intake subsystem with toggleable full-speed operation.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final PWMSparkMax intakeMotor;
    private boolean intakeEnabled = false;

    /** Creates the intake subsystem and applies configured inversion. */
    public IntakeSubsystem(){
        intakeMotor = new PWMSparkMax(IntakeConstants.intakeMotorPort);
        intakeMotor.setInverted(IntakeConstants.intakeMotorReversed);
    }

    /**
     * Starts the intake motor at the requested output.
     *
     * @param speed PWM output in the range [-1, 1]
     */
    public void startIntakeMotor(double speed){
        intakeEnabled = true;
        intakeMotor.set(speed);
    }

    /** Stops the intake motor and clears the enabled flag. */
    public void stopIntakeMotor(){
        intakeEnabled = false;
        intakeMotor.stopMotor();
    }

    /** Toggles the intake between stopped and full-speed operation. */
    public void toggleIntakeMotor() {
        if (intakeEnabled) {
            stopIntakeMotor();
            return;
        }

        startIntakeMotor(IntakeConstants.intakeFullSpeed);
    }

    /**
     * Returns whether the intake is currently enabled.
     *
     * @return {@code true} when the intake is running
     */
    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }
}
