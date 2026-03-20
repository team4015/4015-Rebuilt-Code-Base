// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IndexerCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;

  /**
   * The commands are what make the subsystems actually run. 
   * Every time the scheduler runs, the command does something that makes the subsystem work.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerCommand(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.runIndexer(); 
    System.out.println("Indexer Command Called");
    SmartDashboard.putBoolean("indexerRunning?", true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopIndexer();
    System.out.println("Indexer Command Stopped");
    SmartDashboard.putBoolean("indexerRunning?", false);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
