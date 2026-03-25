package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Instant command that toggles only the indexer motor.
 */
public class IndexerCommand extends InstantCommand {
    private final ShooterSubsystem shooterSubsystem;

    public IndexerCommand(ShooterSubsystem shooterSubsystem) {
        super();
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    /** Toggles the indexer state independently of the shooter flywheel. */
    public void initialize() {
        shooterSubsystem.toggleIndexer();
    }
}

