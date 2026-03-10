package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendableHopperConstants;

/**
 * PWM-controlled hopper subsystem with forward and reverse limit protection.
 */
public class ExtendableHopperSubsystem extends SubsystemBase {
    private final PWMSparkMax extendableHopper;

    private final DigitalInput frontSwitch;
    private final DigitalInput backSwitch;

    /** Creates the hopper subsystem and binds motor and limit switches. */
    public ExtendableHopperSubsystem(){
        extendableHopper = new PWMSparkMax(ExtendableHopperConstants.extendableHopperMotorPort);
        frontSwitch = new DigitalInput(ExtendableHopperConstants.frontLimitSwitch);
        backSwitch = new DigitalInput(ExtendableHopperConstants.backLimitSwitch);
    }

    /**
     * Drives the hopper motor at the requested output.
     *
     * @param speed PWM output in the range [-1, 1]
     */
    public void startExtendableHopperMotor(double speed){
        extendableHopper.set(speed);
    }

    /** Stops the hopper motor. */
    public void stopExtendableHopperMotor(){
        extendableHopper.stopMotor();
    }


    /**
     * Runs the hopper while respecting travel limits.
     *
     * @param speed commanded speed magnitude
     * @param reverseExtendableHopper whether motion should run in reverse
     */
    public void runExtendableHopper(double speed, boolean reverseExtendableHopper){
        if(frontSwitch.get()){
            stopExtendableHopperMotor();
            return;
        }

        if(backSwitch.get()){
            stopExtendableHopperMotor();
            return;
        }

        double appliedSpeed = reverseExtendableHopper ? -speed : speed;
        startExtendableHopperMotor(appliedSpeed);
    }

    /** Runs the hopper forward at match start if the forward limit is clear. */
    public void startForwardAtMatchStart() {
        if (frontSwitch.get()) {
            stopExtendableHopperMotor();
            return;
        }

        runExtendableHopper(ExtendableHopperConstants.matchStartForwardSpeed, false);
    }

    /** Runs the hopper forward at the configured manual speed. */
    public void runForwardManual() {
        runExtendableHopper(ExtendableHopperConstants.manualSpeed, false);
    }

    /** Runs the hopper backward at the configured manual speed. */
    public void runBackwardManual() {
        runExtendableHopper(ExtendableHopperConstants.manualSpeed, true);
    }

}
