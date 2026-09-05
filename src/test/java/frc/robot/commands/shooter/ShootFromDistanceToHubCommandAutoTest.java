package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShootFromDistanceToHubCommandAutoTest {
    @Test
    void feedsOnlyWithValidReadyShotAndNoUnwind() {
        assertTrue(ShootFromDistanceToHubCommandAuto.shouldFeed(true, true, false));
        assertFalse(ShootFromDistanceToHubCommandAuto.shouldFeed(true, true, true));
        assertFalse(ShootFromDistanceToHubCommandAuto.shouldFeed(true, false, false));
        assertFalse(ShootFromDistanceToHubCommandAuto.shouldFeed(false, true, false));
    }

    @Test
    void detectsLongTurretMoveAsUnwind() {
        assertTrue(HubShotController.requiresUnwind(Math.toRadians(170.0), Math.toRadians(-170.0)));
        assertFalse(HubShotController.requiresUnwind(Math.toRadians(10.0), Math.toRadians(20.0)));
        assertFalse(HubShotController.requiresUnwind(0.0, Math.PI));
    }
}
