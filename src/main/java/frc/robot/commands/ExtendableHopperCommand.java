package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.ExtendableHopperSubsystem;

/**
 * Runs the extendable hopper manually while scheduled.
 */
public class ExtendableHopperCommand extends Command {
    private final ExtendableHopperSubsystem extendableHopperSubsystem;
    private final boolean reverse;

    /**
     * Creates a manual hopper command.
     *
     * @param extendableHopperSubsystem hopper subsystem to control
     * @param reverse whether the hopper should run backward instead of forward
     */
    public ExtendableHopperCommand(ExtendableHopperSubsystem extendableHopperSubsystem, boolean reverse) {
        this.extendableHopperSubsystem = extendableHopperSubsystem;
        this.reverse = reverse;
        addRequirements(extendableHopperSubsystem);
    }

    @Override
    /** Drives the hopper in the configured direction. */
    public void execute() {
        if (reverse) {
            extendableHopperSubsystem.runBackwardManual();
        } else {
            extendableHopperSubsystem.runForwardManual();
        }
    }

    @Override
    /**
     * Stops the hopper when the command ends.
     *
     * @param interrupted whether the command was interrupted
     */
    public void end(boolean interrupted) {
        extendableHopperSubsystem.stopExtendableHopperMotor();
    }
}
