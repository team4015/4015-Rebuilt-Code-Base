package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/**
 * Instant command that toggles the intake on or off.
 */
public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates the intake toggle command.
     *
     * @param intakeSubsystem intake subsystem to control
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    /** Toggles the intake state. */
    public void initialize() {
        intakeSubsystem.toggleIntakeMotor();
    }

    @Override
    /**
     * Ends immediately after toggling the intake.
     *
     * @return always {@code true}
     */
    public boolean isFinished() {
        return true;
    }
}
