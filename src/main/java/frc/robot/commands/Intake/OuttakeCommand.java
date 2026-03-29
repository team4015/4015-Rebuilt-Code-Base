package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/**
 * Instant command that toggles the intake motor between full-speed and stopped states.
 */
public class OuttakeCommand extends InstantCommand {
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates the intake toggle command.
     *
     * @param intakeSubsystem intake subsystem to control
     */
    public OuttakeCommand(IntakeSubsystem intakeSubsystem) {
        super();
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    /** Toggles the intake state. */
    public void initialize() {
        intakeSubsystem.toggleIntakeMotorReversed();
        System.out.println("Outtaking");
    }
}
