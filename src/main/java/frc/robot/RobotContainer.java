// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.extendIntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverCtrl = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorCtrl = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Intake intake = new Intake(0.7, 0.1);
  private final Shooter shooter = new Shooter(0.8,0.7);

  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    
    //Configure drivebase command
    Command driveCmd = drivebase.driveCommand(() -> driverCtrl.getLeftX(), () -> -driverCtrl.getLeftY(), () -> driverCtrl.getRightX());  
    drivebase.setDefaultCommand(driveCmd);

    //Configure subsystem commands
    operatorCtrl.leftBumper().toggleOnTrue(new ShooterCommand(shooter));
    operatorCtrl.rightBumper().toggleOnTrue(new IndexerCommand(shooter));
    operatorCtrl.a().toggleOnTrue(new IntakeCommand(intake));
    operatorCtrl.b().whileTrue(new extendIntakeCommand(intake));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
*/
  

  
                                   
                                                            
}
