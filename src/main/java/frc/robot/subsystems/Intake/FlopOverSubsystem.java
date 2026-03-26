package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlopOverConstants;

/**
 * PWM VictorSP-driven "flop over" mechanism that automatically reverses direction
 * when either travel limit switch is hit. Toggled on/off via a bound button.
 */
public class FlopOverSubsystem extends SubsystemBase {
    private final VictorSP motor;
    private final DigitalInput forwardLimit;
    private final DigitalInput backwardLimit;

    private boolean active;
    private Direction direction;

    private enum Direction { CLOCKWISE, COUNTERCLOCKWISE }

    /** Creates the flop-over subsystem using JSON-backed configuration. */
    public FlopOverSubsystem() {
        motor = new VictorSP(FlopOverConstants.motorPwmPort);
        motor.setInverted(FlopOverConstants.motorInverted);

        forwardLimit = new DigitalInput(FlopOverConstants.forwardLimitDio);
        backwardLimit = new DigitalInput(FlopOverConstants.backwardLimitDio);

        direction = FlopOverConstants.startClockwise ? Direction.CLOCKWISE : Direction.COUNTERCLOCKWISE;
        active = false;
    }

    /** Toggles the flop-over mechanism between running and stopped states. */
    public void toggle() {
        active = !active;
        if (!active) {
            stopMotor();
            return;
        }

        direction = FlopOverConstants.startClockwise ? Direction.CLOCKWISE : Direction.COUNTERCLOCKWISE;
        updateMotor();
    }

    /** Stops motion and clears the active flag. */
    public void stop() {
        active = false;
        stopMotor();
    }

    @Override
    public void periodic() {
        if (active) {
            updateMotor();
        } else {
            stopMotor();
        }
        publishTelemetry();
    }

    private void updateMotor() {
        // Flip direction at travel limits.
        if (direction == Direction.CLOCKWISE && isBackwardPressed()) {
            direction = Direction.COUNTERCLOCKWISE;
        } else if (direction == Direction.COUNTERCLOCKWISE && isForwardPressed()) {
            direction = Direction.CLOCKWISE;
        }

        // Safety: if both limits are pressed, hold position.
        if (isForwardPressed() && isBackwardPressed()) {
            stopMotor();
            return;
        }

        double output = direction == Direction.CLOCKWISE
            ? FlopOverConstants.clockwiseOutput
            : -FlopOverConstants.counterClockwiseOutput;
        motor.set(output);
    }

    private boolean isForwardPressed() {
        boolean raw = forwardLimit.get();
        return FlopOverConstants.limitsNormallyClosed ? !raw : raw;
    }

    private boolean isBackwardPressed() {
        boolean raw = backwardLimit.get();
        return FlopOverConstants.limitsNormallyClosed ? !raw : raw;
    }

    private void stopMotor() {
        motor.stopMotor();
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Flop/Active", active);
        SmartDashboard.putString("Flop/Direction", direction.name());
        SmartDashboard.putBoolean("Flop/ForwardLimit", isForwardPressed());
        SmartDashboard.putBoolean("Flop/BackwardLimit", isBackwardPressed());
        SmartDashboard.putNumber("Flop/Output", motor.get());
    }
}
