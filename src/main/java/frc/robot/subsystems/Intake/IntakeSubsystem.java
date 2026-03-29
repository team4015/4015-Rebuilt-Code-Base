package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Single-motor intake subsystem with JSON-backed ID/inversion and simple toggle/start/stop helpers.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final PWMSparkMax intakeMotor;
    private final PWMSparkMax extendMotor;
    
    private boolean intakeEnabled = false;
    private boolean extendRunning = false;
    private DigitalInput frontLimitSwitch = new DigitalInput(0);


    /** Creates the intake subsystem and applies configured inversion. */
    public IntakeSubsystem(){
        intakeMotor = new PWMSparkMax(1);
        extendMotor = new PWMSparkMax(0);
        extendMotor.setInverted(false);
        intakeMotor.setInverted(IntakeConstants.intakeMotorReversed);
    }

    /**
     * Starts the intake motor at the requested output.
     *
     * @param speed PWM output in the range [-1, 1]
     */
    public void startIntakeMotor(double speed){
        setIntakeActive(true, speed);
    }

    public void startIntakeMotorReversed(double speed){
        setIntakeActive(true, -speed);
    }

    public void extendIntake(){
        extendRunning = true;
        extendMotor.set(0.175);
        if(!frontLimitSwitch.get()){
            extendMotor.set(0);
            System.out.println("Intake Extension Stopped");
        }
        
    }

    public void retractIntake(){
        extendRunning = true;
        extendMotor.set(-0.25);
    }

    /** Stops the intake motor and clears the enabled flag. */
    public void stopIntakeMotor(){
        setIntakeActive(false, 0.0);
    }

    public void stopExtend(){
        extendRunning = false;
        extendMotor.set(0);
    }

    /** Toggles the intake between stopped and full-speed operation. */
    public void toggleIntakeMotor() {
        if (intakeEnabled) {
            stopIntakeMotor();
            return;
        }

        startIntakeMotor(IntakeConstants.intakeFullSpeed);
    }

    public void toggleIntakeMotorReversed() {
        if (intakeEnabled) {
            stopIntakeMotor();
            return;
        }

        startIntakeMotorReversed(IntakeConstants.intakeFullSpeed);
    }

    
    /**
     * Returns whether the intake is currently enabled.
     *
     * @return {@code true} when the intake is running
     */
    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }

    @Override
    public void periodic() {
        publishTelemetry();
    }

    private void setIntakeActive(boolean active, double speed) {
        intakeEnabled = active;
        if (active) {
            intakeMotor.set(speed);
            return;
        }

        intakeMotor.stopMotor();
    }

    /** Publishes intake state to SmartDashboard. */
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Intake/Active", intakeEnabled);
        SmartDashboard.putNumber("Intake/Output", intakeMotor.get());
        SmartDashboard.putBoolean("Extend/Active", extendRunning);


    }
}
