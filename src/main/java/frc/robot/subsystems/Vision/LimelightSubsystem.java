package frc.robot.subsystems.Vision;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Limelight interface for AprilTag targeting, aiming, and target-geometry calculations.
 */
public class LimelightSubsystem extends SubsystemBase {
    private static final int LIMELIGHT_POSE_FIELD_COUNT = 6;
    private final NetworkTable table =
        NetworkTableInstance.getDefault().getTable(LimelightConstants.limelightName);

    /**
     * @return true when Limelight has a valid target
     */
    public boolean hasValidTarget() {
        return table.getEntry("tv").getDouble(0.0) > 0.5;
    }

    /**
     * @return horizontal offset from crosshair to target in degrees
     */
    public double getTxDegrees() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return vertical offset from crosshair to target in degrees
     */
    public double getTyDegrees() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /**
     * @return detected AprilTag ID, or -1 if none
     */
    public int getTargetTagId() {
        return (int) table.getEntry("tid").getDouble(-1.0);
    }

    /**
     * Returns Limelight's target pose in robot space when available.
     *
     * @return pose array [x, y, z, pitch, yaw, roll] or empty array if unavailable
     */
    public double[] getTargetPoseRobotSpace() {
        double[] pose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        if (pose.length < LIMELIGHT_POSE_FIELD_COUNT) {
            return new double[0];
        }
        return Arrays.copyOf(pose, LIMELIGHT_POSE_FIELD_COUNT);
    }

    /**
     * Distance estimate using robot-space target pose (preferred).
     *
     * @return horizontal distance in meters, or NaN if unavailable
     */
    public double getDistanceToTagMetersFromPose() {
        double[] pose = getTargetPoseRobotSpace();
        if (pose.length < LIMELIGHT_POSE_FIELD_COUNT) {
            return Double.NaN;
        }
        double xMeters = pose[0];
        double yMeters = pose[1];
        return Math.hypot(xMeters, yMeters);
    }

    /**
     * Trig fallback distance estimate from ty, camera angle, and tag height.
     *
     * @return horizontal distance in meters, or NaN if computation is invalid
     */
    public double getDistanceToTagMetersFromTy() {
        double tyDegrees = getTyDegrees();
        double angleRadians = Math.toRadians(LimelightConstants.mountAngleDegrees + tyDegrees);
        double tan = Math.tan(angleRadians);
        if (Math.abs(tan) < 1e-6) {
            return Double.NaN;
        }
        return (LimelightConstants.defaultTagHeightMeters - LimelightConstants.lensHeightMeters) / tan;
    }

    /**
     * Combined distance estimate. Uses robot-space pose first, then trig fallback.
     *
     * @return best distance estimate in meters, or NaN if no valid target
     */
    public double getDistanceToTagMeters() {
        if (!hasValidTarget()) {
            return Double.NaN;
        }

        double poseDistance = getDistanceToTagMetersFromPose();
        if (Double.isFinite(poseDistance)) {
            return poseDistance;
        }
        return getDistanceToTagMetersFromTy();
    }

    /**
     * Calculates rotation speed command to center target on crosshair.
     *
     * @return angular speed command in rad/s
     */
    public double getAimAngularSpeedRadPerSec() {
        if (!hasValidTarget()) {
            return 0.0;
        }

        double omega = -getTxDegrees() * VisionConstants.aimKp;
        return MathUtil.clamp(
            omega,
            -VisionConstants.aimMaxAngularSpeedRadPerSec,
            VisionConstants.aimMaxAngularSpeedRadPerSec
        );
    }

    /**
     * Returns target translation in robot coordinates.
     *
     * <p>Uses Limelight robot-space pose when available and otherwise falls back to trig using
     * distance and horizontal angle.</p>
     *
     * @return target translation in meters, or NaN values if unavailable
     */
    public Translation2d getTargetTranslationRobotSpace() {
        double[] pose = getTargetPoseRobotSpace();
        if (pose.length >= LIMELIGHT_POSE_FIELD_COUNT) {
            return new Translation2d(pose[0], pose[1]);
        }

        double distanceMeters = getDistanceToTagMeters();
        if (!Double.isFinite(distanceMeters)) {
            return new Translation2d(Double.NaN, Double.NaN);
        }

        Rotation2d txRotation = Rotation2d.fromDegrees(getTxDegrees());
        return new Translation2d(distanceMeters, txRotation);
    }

    /**
     * Returns the estimated hub-center translation in robot coordinates.
     *
     * <p>This adjusts the observed tag translation by configured depth and lateral offsets so the
     * shooter aims toward the scoring center rather than the visible tag edge.</p>
     *
     * @return hub-center translation in meters, or NaN values if unavailable
     */
    public Translation2d getHubCenterTranslationRobotSpace() {
        Translation2d targetTranslation = getTargetTranslationRobotSpace();
        if (!Double.isFinite(targetTranslation.getX()) || !Double.isFinite(targetTranslation.getY())) {
            return targetTranslation;
        }

        Rotation2d targetBearing = targetTranslation.getAngle();
        Translation2d depthOffset = new Translation2d(VisionConstants.hubFrontEdgeToCenterMeters, targetBearing);
        Translation2d lateralOffset = new Translation2d(
            VisionConstants.hubAprilTagLateralOffsetMeters,
            targetBearing.plus(Rotation2d.fromDegrees(90.0))
        );
        return targetTranslation.plus(depthOffset).plus(lateralOffset);
    }

    /**
     * @return true if target is within aiming tolerance
     */
    public boolean isAimedAtTag() {
        return hasValidTarget() && Math.abs(getTxDegrees()) <= VisionConstants.aimToleranceDegrees;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/HasTarget", hasValidTarget());
        SmartDashboard.putNumber("Limelight/TagId", getTargetTagId());
        SmartDashboard.putNumber("Limelight/TxDeg", getTxDegrees());
        SmartDashboard.putNumber("Limelight/TyDeg", getTyDegrees());
        SmartDashboard.putNumber("Limelight/DistanceM", getDistanceToTagMeters());
        SmartDashboard.putBoolean("Limelight/IsAimed", isAimedAtTag());
        SmartDashboard.putNumber("Limelight/HubCenterX", getHubCenterTranslationRobotSpace().getX());
        SmartDashboard.putNumber("Limelight/HubCenterY", getHubCenterTranslationRobotSpace().getY());
    }
}
