package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Runs the hood reset routine until the home limit switch is reached.
 */
public class HoodResetCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Creates the hood reset command.
     *
     * @param shooterSubsystem shooter subsystem that owns the hood
     */
    public HoodResetCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    /** Starts the hood reset routine. */
    public void initialize() {
        shooterSubsystem.beginHoodReset();
    }

    @Override
    /**
     * Finishes once the hood reset routine completes.
     *
     * @return {@code true} when the hood is no longer resetting
     */
    public boolean isFinished() {
        return !shooterSubsystem.isHoodResetting();
    }

    @Override
    /**
     * No-op end hook retained for command lifecycle clarity.
     *
     * @param interrupted whether the command was interrupted
     */
    public void end(boolean interrupted) {
    }
}
