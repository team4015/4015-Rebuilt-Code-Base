// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is primarily used for binding subsystems to commands and commands to controls. 
 * As we have already binded commands to controls in the three commands files:
 *    - DrivetrainCommand.java Lines 35-36
 *    - IntakeCommand.java Line 37
 *    - ShooterCommand.java Line 37
 * We do not need to bind commands to controls here. We just need to bind subsystems to commands.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController ctrl = new XboxController(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake(0.70);
  private final Shooter shooter = new Shooter(0.80);

  private DrivetrainCommand driveCommand = new DrivetrainCommand(drivetrain, ctrl);
  private IntakeCommand intakeCommand = new IntakeCommand(intake, ctrl);
  private ShooterCommand shooterCommand = new ShooterCommand(shooter, ctrl);

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
  }

  /** IF ANYONE IS LOOKING AT THIS CODE RIGHT NOW IGNORE THE FOLLOWING LINES
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   following lines are pretty important*/
  private void configureBindings() {

    // Binding subsystems to commands
    drivetrain.setDefaultCommand(driveCommand);
    intake.setDefaultCommand(intakeCommand);
    shooter.setDefaultCommand(shooterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
