package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

/**
 * Chooses the nearest shot preset (by hub face and distance) and starts the shooter sequence.
 */
public class PresetShootCommand extends InstantCommand {
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    public PresetShootCommand(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        double distanceMeters = limelightSubsystem.getDistanceToTagMeters();
        int tagId = limelightSubsystem.getTargetTagId();

        shooterSubsystem.applyNearestPreset(distanceMeters, tagId);
        shooterSubsystem.startShooterWithFollowerDelay();
    }
}

