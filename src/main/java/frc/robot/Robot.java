package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;

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
    private ShuffleboardTab visionTab;
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    /**
     * Robot-wide initialization hook called once on startup.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        tab = Shuffleboard.getTab("Encoder Offsets");
        visionTab = Shuffleboard.getTab("Vision");
        shooterSubsystem = robotContainer.getShooterSubsystem();
        swerveSubsystem = robotContainer.getSwerveSubsystem();
        LimelightSubsystem limelightSubsystem = robotContainer.getLimelightSubsystem();
        tab.addNumber("Front Left Absolute Encoder", swerveSubsystem::getFrontLeftAbsoluteEncoderRad);
        tab.addNumber("Front Right Absolute Encoder", swerveSubsystem::getFrontRightAbsoluteEncoderRad);
        tab.addNumber("Back Left Absolute Encoder", swerveSubsystem::getBackLeftAbsoluteEncoderRad);
        tab.addNumber("Back Right Absolute Encoder", swerveSubsystem::getBackRightAbsoluteEncoderRad);

        // Ensure turning encoders start aligned to absolute sensors.
        swerveSubsystem.resetModuleEncoders();

        visionTab.addBoolean("Has Target", limelightSubsystem::hasValidTarget);
        visionTab.addNumber("Tag ID", limelightSubsystem::getTargetTagId);
        visionTab.addNumber("TX (deg)", limelightSubsystem::getTxDegrees);
        visionTab.addNumber("Distance (m)", limelightSubsystem::getDistanceToTagMeters);
        visionTab.addBoolean("Aimed", limelightSubsystem::isAimedAtTag);
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
        swerveSubsystem.resetModuleEncoders();
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
        swerveSubsystem.resetModuleEncoders();
        swerveSubsystem.zeroHeading();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * Disabled mode initialization hook.
     */
    @Override
    public void disabledInit() {
        shooterSubsystem.stopAll();
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
