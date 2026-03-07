package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Constants.ShooterConstants;
import org.junit.jupiter.api.Test;

class ShooterTuningTest {
    @Test
    void hoodDegreeConversionRoundTrips() {
        double hoodDeg = 37.5;

        double motorRotations = Shooter.hoodDegreesToMotorRotations(hoodDeg);

        assertEquals(hoodDeg, Shooter.motorRotationsToHoodDegrees(motorRotations), 1.0e-9);
    }

    @Test
    void evaluationUsesConfiguredLinearModel() {
        double distanceMeters = 3.25;

        assertEquals(
                ShooterConstants.HOOD_DISTANCE_SLOPE_DEG_PER_METER * distanceMeters
                        + ShooterConstants.HOOD_DISTANCE_INTERCEPT_DEG,
                Shooter.evaluateHoodDegrees(distanceMeters),
                1.0e-9);
        assertEquals(
                ShooterConstants.FLYWHEEL_DISTANCE_SLOPE_RPS_PER_METER * distanceMeters
                        + ShooterConstants.FLYWHEEL_DISTANCE_INTERCEPT_RPS,
                Shooter.evaluateFlywheelRps(distanceMeters),
                1.0e-9);
    }
}
