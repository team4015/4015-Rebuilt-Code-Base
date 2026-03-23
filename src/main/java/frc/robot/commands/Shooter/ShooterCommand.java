package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Instant command that toggles the coordinated shooter + indexer sequence.
 */
public class ShooterCommand extends InstantCommand {
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Creates the shooter toggle command.
     *
     * @param shooterSubsystem shooter subsystem to control
     */
    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        super();
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    /** Toggles the shooter/indexer state with the configured delay. */
    public void initialize() {
        shooterSubsystem.toggleShooter();
    }
}
