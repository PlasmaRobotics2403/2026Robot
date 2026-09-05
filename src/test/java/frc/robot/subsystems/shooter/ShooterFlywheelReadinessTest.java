package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterFlywheelReadinessTest {
    private static final double TARGET_RPS = 50.0;
    private static final double TOLERANCE_RPS = 2.0;

    @Test
    void acceptsBothMotorsAtToleranceBoundaries() {
        assertTrue(Shooter.areFlywheelsWithinTolerance(true, 48.0, 52.0, TARGET_RPS, TOLERANCE_RPS));
    }

    @Test
    void rejectsEitherMotorOutsideTolerance() {
        assertFalse(Shooter.areFlywheelsWithinTolerance(true, 47.99, 50.0, TARGET_RPS, TOLERANCE_RPS));
        assertFalse(Shooter.areFlywheelsWithinTolerance(true, 50.0, 52.01, TARGET_RPS, TOLERANCE_RPS));
    }

    @Test
    void rejectsAverageThatHidesFailedMotor() {
        assertFalse(Shooter.areFlywheelsWithinTolerance(true, 0.0, 100.0, TARGET_RPS, TOLERANCE_RPS));
    }

    @Test
    void rejectsDisconnectedOrInvalidMeasurements() {
        assertFalse(Shooter.areFlywheelsWithinTolerance(false, 50.0, 50.0, TARGET_RPS, TOLERANCE_RPS));
        assertFalse(Shooter.areFlywheelsWithinTolerance(true, Double.NaN, 50.0, TARGET_RPS, TOLERANCE_RPS));
        assertFalse(
                Shooter.areFlywheelsWithinTolerance(true, 50.0, Double.POSITIVE_INFINITY, TARGET_RPS, TOLERANCE_RPS));
        assertFalse(Shooter.areFlywheelsWithinTolerance(true, 50.0, 50.0, TARGET_RPS, -1.0));
    }
}
