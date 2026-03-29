package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/**
 * Drives the intake arm up or down. When moving down, the lower limit switch
 * will stop the motor automatically to protect the mechanism.
 */
public class MoveIntakeArmCommand extends Command {
    public enum Direction { DOWN, UP }

    private final IntakeSubsystem intake;
    private final Direction direction;

    public MoveIntakeArmCommand(IntakeSubsystem intake, Direction direction) {
        this.intake = intake;
        this.direction = direction;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("IntakeArm/Command", direction.name());
        if (direction == Direction.DOWN) {
            // If we're already at the lower stop, don't drive further.
            if (intake.isLowerLimitPressed()) {
                intake.stopExtend();
                return;
            }
            intake.extendIntake();
        } else {
            intake.retractIntake();
        }
    }

    @Override
    public void execute() {
        if (direction == Direction.DOWN && intake.isLowerLimitPressed()) {
            intake.stopExtend();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopExtend();
        SmartDashboard.putString("IntakeArm/Command", "Stopped");
    }

    @Override
    public boolean isFinished() {
        return direction == Direction.DOWN && intake.isLowerLimitPressed();
    }
}
