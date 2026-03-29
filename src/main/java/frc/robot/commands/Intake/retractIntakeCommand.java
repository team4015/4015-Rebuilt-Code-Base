package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs the intake arm inward until interrupted. Restores the earlier retract command.
 */
public class retractIntakeCommand extends Command {
  private final IntakeSubsystem intake;

  public retractIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.retractIntake();
    System.out.println("Intake Retracting");
    SmartDashboard.putBoolean("intakeRetracting", true);
    SmartDashboard.putBoolean("intakeExtending?", false);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtend();
    System.out.println("Extending Stopped");
    SmartDashboard.putBoolean("intakeRetracting", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
