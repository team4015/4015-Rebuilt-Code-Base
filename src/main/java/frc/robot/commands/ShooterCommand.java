// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;

  /**
   * The commands are what make the subsystems actually run. 
   * Every time the scheduler runs, the command does something that makes the subsystem work.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.runShooter(); 
    System.out.println("Shooter Command Called");
    SmartDashboard.putBoolean("shooterRunning?", true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    System.out.println("Shooter Command Stopped");
    SmartDashboard.putBoolean("shooterRunning?", false);
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
