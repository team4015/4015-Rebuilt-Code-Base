// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeIOSim;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final CommandPS4Controller driverCtrl = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller operatorCtrl = new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  
  private final PathPlannerAuto auto;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("intake", new IntakeCommand(intake));
    NamedCommands.registerCommand("extendIntake", new extendIntakeCommand(intake));
    NamedCommands.registerCommand("shoot", new ShooterCommand(shooter));
    NamedCommands.registerCommand("index", new IndexerCommand(shooter));

    auto = new PathPlannerAuto("rebuiltAuto");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);



    configureBindings();
  }

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverCtrl.getLeftY() * -1,
                  () -> driverCtrl.getLeftX() * 1)
          .withControllerRotationAxis(() -> -driverCtrl.getRightX())
          .deadband(Constants.OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  public SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverCtrl.getLeftX() * 1,
                  () -> -driverCtrl.getLeftY() * -1)
          .withControllerRotationAxis(() -> -driverCtrl.getRightX())
          .deadband(Constants.OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true);
                        
  public SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
                                                    .robotRelative(true)
                                                    .allianceRelativeControl(false);


  private void configureBindings() {
    //Configure drivebase command
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAngularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    WaitCommand wait = new WaitCommand(1.25);

    Command driveRobotOrientedAngularVelocity = drivebase.drive(driveRobotOriented);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocitySim);

    //Configure subsystem commands
   
    driverCtrl.L1().toggleOnTrue(new IntakeCommand(intake));
    driverCtrl.R1().toggleOnTrue(new ShooterCommand(shooter)
            .alongWith(wait
                          .beforeStarting(
                                  () -> SmartDashboard.putBoolean("Waiting?", true)
                          ).finallyDo(
                                  () -> SmartDashboard.putBoolean("Waiting?", false)
                          ).andThen(
                                  new IndexerCommand(shooter)
                          )
            )
    );
    driverCtrl.square().toggleOnTrue(new OuttakeCommand(intake));
    driverCtrl.L2().toggleOnTrue(new extendIntakeCommand(intake));
    driverCtrl.R2().toggleOnTrue(new retractIntakeCommand(intake));

    driverCtrl.circle().toggleOnTrue(driveRobotOrientedAngularVelocity
                                    .beforeStarting(() -> SmartDashboard.putBoolean("isRobotOriented", true))
                                    .finallyDo(() -> SmartDashboard.putBoolean("isRobotOriented", false)));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    System.out.println("Autonomous Command Called!!!");
    return auto;
  }
}
