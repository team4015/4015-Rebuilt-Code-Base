package frc.robot.commands.FlopOver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlopOver.FlopOverSubsystem;

/**
 * Drives the flop-over arm down until the down limit switch is reached, then ends.
 */
public class FlopOverCommand extends Command {
    private final FlopOverSubsystem flopOverSubsystem;

    public FlopOverCommand(FlopOverSubsystem flopOverSubsystem) {
        this.flopOverSubsystem = flopOverSubsystem;
        addRequirements(flopOverSubsystem);
    }

    @Override
    public void initialize() {
        flopOverSubsystem.driveDown();
    }

    @Override
    public void execute() {
        flopOverSubsystem.driveDown();
    }

    @Override
    public void end(boolean interrupted) {
        flopOverSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return flopOverSubsystem.isDown();
    }
}
