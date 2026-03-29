package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs the intake arm outward until interrupted. Mirrors the behavior we had on the
 * GeorgianCodeV6 branch (pre MoveIntakeArmCommand).
 */
public class extendIntakeCommand extends Command {
  private final IntakeSubsystem intake;

  public extendIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  @Override
  public void initialize(){
    intake.extendIntake();
    System.out.println("Intake Extending");
    SmartDashboard.putBoolean("intakeExtending?", true);
  }

  @Override
  public void execute() {
    // No periodic work required; SparkMAX handles the constant output.
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopExtend();
    System.out.println("Extending Stopped");
    SmartDashboard.putBoolean("intakeExtending?", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
