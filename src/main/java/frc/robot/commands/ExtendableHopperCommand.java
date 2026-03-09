package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtendableHopperSubsystem;

public class ExtendableHopperCommand extends Command {
    private final ExtendableHopperSubsystem extendableHopperSubsystem;
    private final boolean reverse;

    public ExtendableHopperCommand(ExtendableHopperSubsystem extendableHopperSubsystem, boolean reverse) {
        this.extendableHopperSubsystem = extendableHopperSubsystem;
        this.reverse = reverse;
        addRequirements(extendableHopperSubsystem);
    }

    @Override
    public void execute() {
        if (reverse) {
            extendableHopperSubsystem.runBackwardManual();
        } else {
            extendableHopperSubsystem.runForwardManual();
        }
    }

    @Override
    public void end(boolean interrupted) {
        extendableHopperSubsystem.stopExtendableHopperMotor();
    }
}
