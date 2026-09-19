// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController driverCtrl = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorCtrl = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  
  private final PathPlannerAuto auto;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("intake", runIntake);
    NamedCommands.registerCommand("extendIntake", extendIntake);
    NamedCommands.registerCommand("shoot", runShooter);
    NamedCommands.registerCommand("index", runIndexer);

    auto = new PathPlannerAuto("baneOfMyExistence");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

       // drivebase.getSwerveDrive().getSwerveController().addSlewRateLimiters(limiter, limiter, limiter);


  }

  


  //Intake Commands
  Command runIntake = intake.runIntake2();
  Command runOuttake = intake.runOuttake2();
  Command extendIntake = intake.extendIntake2();
  Command retractIntake = intake.retractIntake2();

  //Shooter Commands
  Command runShooter = shooter.runShooter2();
  Command runIndexer = shooter.runIndexer2();

  

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  public SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverCtrl.getLeftX() * 0.2,
                  () -> driverCtrl.getLeftY() * 0.2)
          .withControllerRotationAxis(() -> -driverCtrl.getRightX() * 0.5)

          .deadband(Constants.OperatorConstants.DEADBAND)
          .scaleTranslation(1)
          .allianceRelativeControl(true)
          .cubeTranslationControllerAxis(false)
          .scaleRotation(0.6);

  public SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -driverCtrl.getLeftX() * 1,
                  () -> -driverCtrl.getLeftY() * 1)
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


    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    //Configure subsystem commands

    driverCtrl.leftBumper().toggleOnTrue(runIntake);
    driverCtrl.rightBumper().toggleOnTrue(runShooter
            .alongWith(wait
                          .beforeStarting(
                                  () -> SmartDashboard.putBoolean("Waiting?", true)
                          ).finallyDo(
                                  () -> SmartDashboard.putBoolean("Waiting?", false)
                          ).andThen(
                                  runIndexer
                          )
            )
    );
    driverCtrl.a().toggleOnTrue(runOuttake);
    driverCtrl.leftTrigger().toggleOnTrue(extendIntake);
    driverCtrl.rightTrigger().toggleOnTrue(retractIntake);

    //driverCtrl.b().toggleOnTrue(driveRobotOrientedAngularVelocitySim
      //                              .beforeStarting(() -> SmartDashboard.putBoolean("isRobotOriented", true))
        //                            .finallyDo(() -> SmartDashboard.putBoolean("isRobotOriented", false)));
   
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
