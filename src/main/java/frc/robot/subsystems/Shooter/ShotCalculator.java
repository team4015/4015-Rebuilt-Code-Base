package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Utility that computes hood angle and yaw lead for motion-compensated shooting.
 */
final class ShotCalculator {
    private ShotCalculator() {
    }

    /**
     * Solves for a hood angle and yaw lead that minimize projectile miss distance.
     *
     * @param inputs projectile and target inputs
     * @return best shot solution found, or an invalid solution when none is found
     */
    public static ShotSolution solve(ShotInputs inputs) {
        if (!inputs.isValid()) {
            return ShotSolution.noSolution();
        }

        ShotSolution best = ShotSolution.noSolution();
        double coarseStep = Math.toRadians(0.5);
        for (double angle = inputs.minHoodAngleRad; angle <= inputs.maxHoodAngleRad; angle += coarseStep) {
            ShotSolution candidate = simulateForAngle(inputs, angle);
            if (candidate.isBetterThan(best)) {
                best = candidate;
            }
        }

        if (!best.valid) {
            return best;
        }

        double fineStep = Math.toRadians(0.05);
        double start = Math.max(inputs.minHoodAngleRad, best.hoodAngleRad - Math.toRadians(1.0));
        double end = Math.min(inputs.maxHoodAngleRad, best.hoodAngleRad + Math.toRadians(1.0));
        for (double angle = start; angle <= end; angle += fineStep) {
            ShotSolution candidate = simulateForAngle(inputs, angle);
            if (candidate.isBetterThan(best)) {
                best = candidate;
            }
        }

        return best;
    }

    private static ShotSolution simulateForAngle(ShotInputs inputs, double hoodAngleRad) {
        double horizontalLaunchSpeed = inputs.launchSpeedMetersPerSecond * Math.cos(hoodAngleRad);
        if (horizontalLaunchSpeed <= 1e-6) {
            return ShotSolution.noSolution();
        }

        Translation2d targetDirection = inputs.targetTranslationMeters.times(1.0 / inputs.targetTranslationMeters.getNorm());
        Translation2d perpendicular = new Translation2d(-targetDirection.getY(), targetDirection.getX());
        double lateralRobotSpeed = dot(inputs.predictedRobotVelocityMetersPerSecond, perpendicular);
        double yawLeadRad = Math.asin(MathUtil.clamp(-lateralRobotSpeed / horizontalLaunchSpeed, -0.95, 0.95));
        Translation2d shotDirection = targetDirection.rotateBy(Rotation2d.fromRadians(yawLeadRad));

        double vx = inputs.predictedRobotVelocityMetersPerSecond.getX() + shotDirection.getX() * horizontalLaunchSpeed;
        double vy = inputs.predictedRobotVelocityMetersPerSecond.getY() + shotDirection.getY() * horizontalLaunchSpeed;
        double vz = inputs.launchSpeedMetersPerSecond * Math.sin(hoodAngleRad);

        double x = 0.0;
        double y = 0.0;
        double z = inputs.releaseHeightMeters;
        double bestErrorMeters = Double.POSITIVE_INFINITY;
        double bestTimeSeconds = 0.0;

        for (double t = 0.0; t <= inputs.maxTimeSeconds; t += inputs.timeStepSeconds) {
            double error = Math.sqrt(
                square(inputs.targetTranslationMeters.getX() - x)
                    + square(inputs.targetTranslationMeters.getY() - y)
                    + square(inputs.targetHeightMeters - z)
            );

            if (error < bestErrorMeters) {
                bestErrorMeters = error;
                bestTimeSeconds = t;
            }

            if (z < 0.0 && t > 0.1) {
                break;
            }

            double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
            double dragAccelScale = speed > 1e-6
                ? (0.5 * inputs.airDensityKgPerCubicMeter * inputs.dragCoefficient * inputs.crossSectionAreaMetersSquared
                    / inputs.ballMassKg) * speed
                : 0.0;

            double ax = -dragAccelScale * vx;
            double ay = -dragAccelScale * vy;
            double az = -inputs.gravityMetersPerSecondSquared - (dragAccelScale * vz);

            vx += ax * inputs.timeStepSeconds;
            vy += ay * inputs.timeStepSeconds;
            vz += az * inputs.timeStepSeconds;

            x += vx * inputs.timeStepSeconds;
            y += vy * inputs.timeStepSeconds;
            z += vz * inputs.timeStepSeconds;
        }

        return new ShotSolution(true, hoodAngleRad, yawLeadRad, bestTimeSeconds, bestErrorMeters);
    }

    private static double dot(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX()) + (a.getY() * b.getY());
    }

    private static double square(double value) {
        return value * value;
    }

    /** Immutable set of values required by the shot solver. */
    static final class ShotInputs {
        final Translation2d targetTranslationMeters;
        final Translation2d predictedRobotVelocityMetersPerSecond;
        final double releaseHeightMeters;
        final double targetHeightMeters;
        final double launchSpeedMetersPerSecond;
        final double ballMassKg;
        final double crossSectionAreaMetersSquared;
        final double dragCoefficient;
        final double airDensityKgPerCubicMeter;
        final double gravityMetersPerSecondSquared;
        final double minHoodAngleRad;
        final double maxHoodAngleRad;
        final double timeStepSeconds;
        final double maxTimeSeconds;

        ShotInputs(
            Translation2d targetTranslationMeters,
            Translation2d predictedRobotVelocityMetersPerSecond,
            double releaseHeightMeters,
            double targetHeightMeters,
            double launchSpeedMetersPerSecond,
            double ballMassKg,
            double crossSectionAreaMetersSquared,
            double dragCoefficient,
            double airDensityKgPerCubicMeter,
            double gravityMetersPerSecondSquared,
            double minHoodAngleRad,
            double maxHoodAngleRad,
            double timeStepSeconds,
            double maxTimeSeconds
        ) {
            this.targetTranslationMeters = targetTranslationMeters;
            this.predictedRobotVelocityMetersPerSecond = predictedRobotVelocityMetersPerSecond;
            this.releaseHeightMeters = releaseHeightMeters;
            this.targetHeightMeters = targetHeightMeters;
            this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
            this.ballMassKg = ballMassKg;
            this.crossSectionAreaMetersSquared = crossSectionAreaMetersSquared;
            this.dragCoefficient = dragCoefficient;
            this.airDensityKgPerCubicMeter = airDensityKgPerCubicMeter;
            this.gravityMetersPerSecondSquared = gravityMetersPerSecondSquared;
            this.minHoodAngleRad = minHoodAngleRad;
            this.maxHoodAngleRad = maxHoodAngleRad;
            this.timeStepSeconds = timeStepSeconds;
            this.maxTimeSeconds = maxTimeSeconds;
        }

        boolean isValid() {
            return targetTranslationMeters != null
                && Double.isFinite(targetTranslationMeters.getX())
                && Double.isFinite(targetTranslationMeters.getY())
                && targetTranslationMeters.getNorm() > 1e-6
                && Double.isFinite(launchSpeedMetersPerSecond)
                && launchSpeedMetersPerSecond > 0.0
                && Double.isFinite(targetHeightMeters)
                && Double.isFinite(releaseHeightMeters);
        }
    }

    /** Result produced by the shot solver. */
    static final class ShotSolution {
        static final ShotSolution NO_SOLUTION = new ShotSolution(false, 0.0, 0.0, 0.0, Double.POSITIVE_INFINITY);

        final boolean valid;
        final double hoodAngleRad;
        final double yawLeadRad;
        final double flightTimeSeconds;
        final double missDistanceMeters;

        ShotSolution(
            boolean valid,
            double hoodAngleRad,
            double yawLeadRad,
            double flightTimeSeconds,
            double missDistanceMeters
        ) {
            this.valid = valid;
            this.hoodAngleRad = hoodAngleRad;
            this.yawLeadRad = yawLeadRad;
            this.flightTimeSeconds = flightTimeSeconds;
            this.missDistanceMeters = missDistanceMeters;
        }

        static ShotSolution noSolution() {
            return NO_SOLUTION;
        }

        boolean isBetterThan(ShotSolution other) {
            return valid && (!other.valid || missDistanceMeters < other.missDistanceMeters);
        }
    }
}
