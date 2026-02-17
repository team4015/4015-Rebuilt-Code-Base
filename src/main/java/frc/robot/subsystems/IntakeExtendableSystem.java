package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeExtendableConstants;

public class IntakeExtendableSystem extends SubsystemBase {
    private SparkMax intakeMotor;
    private SparkMax extendableHopperMotor;

    private SparkMaxConfig intakeConfig;
    private SparkMaxConfig extendableHopperConfig;

    private DigitalInput frontSwitch;
    private DigitalInput backSwitch;

    private boolean reverseExtendableHopper;
    private final boolean isReveresedIntakeMotor = true;
    private final boolean isReverseExtendableHopper = true;

    public IntakeExtendableSystem(){
        intakeMotor = new SparkMax(IntakeExtendableConstants.intakeMotorPort, MotorType.kBrushless);
        extendableHopperMotor = new SparkMax(IntakeExtendableConstants.extendableHopperMotorPort, MotorType.kBrushless);

        intakeConfig = new SparkMaxConfig();
        extendableHopperConfig = new SparkMaxConfig();

        frontSwitch = new DigitalInput(IntakeExtendableConstants.frontLimitSwitch);
        backSwitch = new DigitalInput(IntakeExtendableConstants.backLimitSwitch);

        reverseExtendableHopper = false;

        intakeConfig
                .inverted(isReveresedIntakeMotor)
                .idleMode(IdleMode.kBrake);

        extendableHopperConfig
                .inverted(isReverseExtendableHopper)
                .idleMode(IdleMode.kBrake);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extendableHopperMotor.configure(extendableHopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startIntakeMotor(double speed){
        intakeMotor.set(speed);
    }

    public void startExtendableHopperMotor(double speed){
        extendableHopperMotor.set(speed);
    }

    public void stopIntakeMotor(){
        intakeMotor.stopMotor();
    }

    public void stopExtendableHopperMotor(){
        extendableHopperMotor.stopMotor();
    }

    public void runExtendableHopper(double speed){
        speed = (reverseExtendableHopper) ? -speed: speed;
        startExtendableHopperMotor(speed);

        if(frontSwitch.get()){
            stopExtendableHopperMotor();
            speed *= -1.0;
        }

        if(backSwitch.get()){
            stopExtendableHopperMotor();
            speed *= 1.0;
        }
    }
}
