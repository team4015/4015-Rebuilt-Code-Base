package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Instant command that toggles the shooter and indexer together.
 */
public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Creates the shooter toggle command.
     *
     * @param shooterSubsystem shooter subsystem to control
     */
    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    /** Toggles the shooter state. */
    public void initialize() {
        shooterSubsystem.toggleShooting();
    }

    @Override
    /**
     * Ends immediately after toggling the shooter.
     *
     * @return always {@code true}
     */
    public boolean isFinished() {
        return true;
    }
}
