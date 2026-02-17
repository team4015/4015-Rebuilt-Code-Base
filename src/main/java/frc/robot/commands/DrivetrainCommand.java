// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that runs the drivetrain. */
public class DrivetrainCommand extends Command {
  private Drivetrain drivetrain;
  private XboxController ctrl;

  /**
   * The commands are what make the subsystems actually run. 
   * Every time the scheduler runs, the command does something that makes the subsystem work.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainCommand(Drivetrain drivetrain, XboxController ctrl) {
    this.drivetrain = drivetrain;
    this.ctrl = ctrl;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = ctrl.getRawAxis(1);
    double rightSpeed = ctrl.getRawAxis(5);
    drivetrain.drive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
