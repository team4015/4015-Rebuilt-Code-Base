package frc.robot.subsystems.FlopOver;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlopOverConstants;

/**
 * Controls the flop-over arm that pivots the intake down at match start.
 * Runs the PWM SparkMax in one direction until the down-limit switch is hit, then stops.
 */
public class FlopOverSubsystem extends SubsystemBase {
    private final PWMSparkMax motor;
    private final DigitalInput downLimitSwitch;
    private boolean drivingDown = false;

    public FlopOverSubsystem() {
        motor = new PWMSparkMax(FlopOverConstants.motorPort);
        motor.setInverted(FlopOverConstants.motorReversed);
        downLimitSwitch = new DigitalInput(FlopOverConstants.limitSwitchDioPort);
    }

    /** Begin driving the arm down unless it's already down. */
    public void driveDown() {
        if (isDown()) {
            stop();
            return;
        }
        drivingDown = true;
        motor.set(FlopOverConstants.deployDownSpeed);
    }

    /** Stop all motion and clear the drive flag. */
    public void stop() {
        drivingDown = false;
        motor.stopMotor();
    }

    /** Returns true when the down limit switch is triggered. */
    public boolean isDown() {
        boolean raw = downLimitSwitch.get();
        return FlopOverConstants.limitSwitchActiveHigh ? raw : !raw;
    }

    @Override
    public void periodic() {
        if (drivingDown && isDown()) {
            stop();
        }
        publishTelemetry();
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("FlopOver/DrivingDown", drivingDown);
        SmartDashboard.putBoolean("FlopOver/IsDown", isDown());
        SmartDashboard.putNumber("FlopOver/MotorOutput", motor.get());
    }
}
