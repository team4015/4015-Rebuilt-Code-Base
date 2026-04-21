// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class OuttakeCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Intake intake;


    /**
     * The commands are what make the subsystems actually run.
     * Every time the scheduler runs, the command does something that makes the subsystem work.
     *
     * @param subsystem The subsystem used by this command.
     */
    public OuttakeCommand(Intake intake) {
      this.intake = intake;
      
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runOuttake();
    System.out.println("Outtake Command Called");
    SmartDashboard.putBoolean("Outtaking?", true);
    SmartDashboard.putBoolean("IntakeRunning?", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    System.out.println("Intake Command Stopped");
    SmartDashboard.putBoolean("Outtaking?", false);
    SmartDashboard.putBoolean("IntakeRunning?", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
