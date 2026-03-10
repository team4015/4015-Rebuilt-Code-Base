package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Main robot lifecycle class.
 *
 * <p>Owns command scheduler execution and mode transitions (autonomous/teleop/test). Also
 * publishes key swerve absolute encoder values to Shuffleboard for quick operator visibility.</p>
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private ShuffleboardTab tab;

    /**
     * Robot-wide initialization hook called once on startup.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        tab = Shuffleboard.getTab("Encoder Offsets");

        SwerveSubsystem swerveSubsystem = robotContainer.getSwerveSubsystem();
        tab.addNumber("Front Left Absolute Encoder", swerveSubsystem::getFrontLeftAbsoluteEncoderRad);
        tab.addNumber("Front Right Absolute Encoder", swerveSubsystem::getFrontRightAbsoluteEncoderRad);
        tab.addNumber("Back Left Absolute Encoder", swerveSubsystem::getBackLeftAbsoluteEncoderRad);
        tab.addNumber("Back Right Absolute Encoder", swerveSubsystem::getBackRightAbsoluteEncoderRad);
    }

    /**
     * Runs every robot loop. Executes the command scheduler.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules autonomous command when autonomous mode begins.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /**
     * Cancels autonomous command when teleop starts.
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * Disabled mode initialization hook.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * Autonomous periodic hook.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Teleop periodic hook.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * Disabled periodic hook.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * Test mode initialization hook. Clears all scheduled commands.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Test periodic hook.
     */
    @Override
    public void testPeriodic() {
    }
}
