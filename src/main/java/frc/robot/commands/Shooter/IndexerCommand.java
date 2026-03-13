package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Instant command that toggles the indexer state.
 */
public class IndexerCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Creates the indexer toggle command.
     *
     * @param shooterSubsystem shooter subsystem that owns the indexer motor
     */
    public IndexerCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    /** Toggles the indexer state. */
    public void initialize() {
        shooterSubsystem.toggleIndexer();
    }

    @Override
    /**
     * Ends immediately after toggling the indexer.
     *
     * @return always {@code true}
     */
    public boolean isFinished() {
        return true;
    }
}
