// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static edu.wpi.first.units.Units.Inches;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer robotContainer;
  private SwerveDriveSimulation swerveDriveSimulation; // ← add here
  private Timer matchTimer = new Timer();

  private boolean autoStarted = false;
  private boolean teleOpStarted = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    System.out.println("RobotContainer Constructor");
    robotContainer = new RobotContainer();
    m_autonomousCommand = robotContainer.getAutonomousCommand();
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */



  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);

    }
  }


  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    matchTimer.reset();
    matchTimer.start();

    autoStarted = false;
    teleOpStarted = false;

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Match Time", matchTimer.get());
    if(!autoStarted){
      autonomousInit();
    }
    if(matchTimer.get() > 20 && !teleOpStarted){
      teleopInit();
      teleOpStarted = true;
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().placeGamePiecesOnField();


    


    /*final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofNav2X())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(COTS.ofMark4i(
                    DCMotor.getNEO(1), // Drive motor is a Kraken X60
                    DCMotor.getNEO(1), // Steer motor is a Falcon 500
                    COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                    2)) // L2 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(Inches.of(21.75), Inches.of(21.75))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(32.755906), Inches.of(32.519685));

    swerveDriveSimulation = new SwerveDriveSimulation(
            driveTrainSimulationConfig,
            new Pose2d(3, 3, new Rotation2d())
    );

    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);*/

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d[] fuelPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Fuel");
    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);

  }
}
