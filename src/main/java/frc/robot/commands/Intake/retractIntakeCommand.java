// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class retractIntakeCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeSubsystem intake;
  
    /**
     * The commands are what make the subsystems actually run. 
     * Every time the scheduler runs, the command does something that makes the subsystem work.
     *
     * @param subsystem The subsystem used by this command.
     */
    public retractIntakeCommand(IntakeSubsystem intake) {
      this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.retractIntake();
    System.out.println("Intake Retracting");
    SmartDashboard.putBoolean("intakeRetracting", true);
    SmartDashboard.putBoolean("intakeExtending?", false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopExtend();
    System.out.println("Extending Stopped");
    SmartDashboard.putBoolean("intakeRetracting", false);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
