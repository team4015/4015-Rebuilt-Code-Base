package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.FlopOverSubsystem;

/**
 * Instant command that toggles the flop-over mechanism on/off.
 */
public class FlopOverCommand extends InstantCommand {
    private final FlopOverSubsystem flopOverSubsystem;

    public FlopOverCommand(FlopOverSubsystem flopOverSubsystem) {
        super();
        this.flopOverSubsystem = flopOverSubsystem;
        addRequirements(flopOverSubsystem);
    }

    @Override
    public void initialize() {
        flopOverSubsystem.toggle();
    }
}
