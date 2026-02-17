package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;
    private SparkMaxConfig intakeConfig;
    private final boolean isReveresedIntakeMotor = true;

    public IntakeSubsystem(){
        intakeMotor = new SparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);

        intakeConfig = new SparkMaxConfig();

        intakeConfig
                .inverted(isReveresedIntakeMotor)
                .idleMode(IdleMode.kBrake);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startIntakeMotor(double speed){
        intakeMotor.set(speed);
    }


    public void stopIntakeMotor(){
        intakeMotor.stopMotor();
    }
}
