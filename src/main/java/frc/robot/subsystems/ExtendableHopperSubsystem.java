package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendableHopperConstants;

public class ExtendableHopperSubsystem extends SubsystemBase {
    private final PWMSparkMax extendableHopper;

    private final DigitalInput frontSwitch;
    private final DigitalInput backSwitch;

    boolean shouldReverseExtendableHopper = true;

    public ExtendableHopperSubsystem(){
        extendableHopper = new PWMSparkMax(ExtendableHopperConstants.extendableHopperMotorPort);
        frontSwitch = new DigitalInput(ExtendableHopperConstants.frontLimitSwitch);
        backSwitch = new DigitalInput(ExtendableHopperConstants.backLimitSwitch);

    }

    public void startExtendableHopperMotor(double speed){
        extendableHopper.set(speed);
    }

    public void stopExtendableHopperMotor(){
        extendableHopper.stopMotor();
    }


    public void runExtendableHopper(double speed, boolean reverseExtendableHopper){
        if(frontSwitch.get()){
            stopExtendableHopperMotor();
            reverseExtendableHopper = true;
            return;
        }

        if(backSwitch.get()){
            stopExtendableHopperMotor();
            reverseExtendableHopper = false;
            return;
        }

        ExtendableHopperConstants.extendableHopperSpeed = (reverseExtendableHopper) ? -speed: speed;
        startExtendableHopperMotor(ExtendableHopperConstants.extendableHopperSpeed );
    }

    public void startForwardAtMatchStart() {
        if (frontSwitch.get()) {
            stopExtendableHopperMotor();
            return;
        }

        ExtendableHopperConstants.reverseExtendableHopper = false;
        runExtendableHopper(ExtendableHopperConstants.matchStartForwardSpeed, false);
    }

    public void runForwardManual() {
        ExtendableHopperConstants.reverseExtendableHopper = false;
        runExtendableHopper(ExtendableHopperConstants.manualSpeed, false);
    }

    public void runBackwardManual() {
        ExtendableHopperConstants.reverseExtendableHopper = true;
        runExtendableHopper(ExtendableHopperConstants.manualSpeed, true);
    }

}
