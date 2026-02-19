package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ExtendableHopperConstants;

public class ExtendableHopperSubsystem {
    private SparkMax extendableHopper;
    private SparkMaxConfig extendableHopperConfig;

    private DigitalInput frontSwitch;
    private DigitalInput backSwitch;

    boolean shouldReverseExtendableHopper = true;

    public ExtendableHopperSubsystem(){
        extendableHopper = new SparkMax(ExtendableHopperConstants.extendableHopperMotorPort, MotorType.kBrushless);
        frontSwitch = new DigitalInput(ExtendableHopperConstants.frontLimitSwitch);
        backSwitch = new DigitalInput(ExtendableHopperConstants.backLimitSwitch);

        extendableHopperConfig = new SparkMaxConfig();

        extendableHopperConfig
                .inverted(shouldReverseExtendableHopper)
                .idleMode(IdleMode.kBrake);

        extendableHopper.configure(extendableHopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
}
