package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class AutoShootEventsTest {
    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    void eventShotRunsInsideAutoAndStopsOnCancellation() {
        Subsystem shooter = new Subsystem() {};
        TrackingCommand shot = new TrackingCommand(shooter);
        TrackingCommand path = new TrackingCommand(new Subsystem() {});
        AutoShootEvents events = new AutoShootEvents(shot);
        Command auto = events.wrapAuto(path);
        assertTrue(auto.getRequirements().contains(shooter));
        auto.initialize();
        events.requestStart();
        auto.execute();
        auto.execute();
        assertEquals(2, path.updates);
        assertEquals(2, shot.updates);
        assertEquals(1, shot.starts);
        auto.end(true);
        assertEquals(1, shot.stops);
        assertEquals(1, path.stops);
        events.requestStart();
        assertEquals(1, shot.starts);
    }

    @Test
    void stopAndRestartDoNotRestartPath() {
        TrackingCommand shot = new TrackingCommand(new Subsystem() {});
        TrackingCommand path = new TrackingCommand(new Subsystem() {});
        AutoShootEvents events = new AutoShootEvents(shot);
        Command auto = events.wrapAuto(path);
        auto.initialize();
        events.requestStart();
        auto.execute();
        events.requestStop();
        auto.execute();
        assertEquals(1, shot.updates);
        events.requestStart();
        auto.execute();
        assertEquals(2, shot.starts);
        assertEquals(1, path.starts);
        auto.end(false);
        assertEquals(2, shot.stops);
    }

    @Test
    void namedMechanismCommandTakesPriorityAndCanFinishBeforePendingEvent() {
        Subsystem shooter = new Subsystem() {};
        TrackingCommand shot = new TrackingCommand(shooter);
        AutoShootEvents events = new AutoShootEvents(shot);
        Command auto = events.wrapAuto(Commands.idle());
        Command named = events.wrapNamedCommand(new TrackingCommand(shooter));
        auto.initialize();
        events.requestStart();
        auto.execute();
        named.initialize();
        assertEquals(1, shot.stops);
        events.requestStart();
        auto.execute();
        assertEquals(1, shot.starts);
        named.end(false);
        auto.execute();
        assertEquals(2, shot.starts);
        auto.end(false);
    }

    @Test
    void sameSelectedAutoCanRunAgainWithoutCompositionOrStateLeak() {
        TrackingCommand shot = new TrackingCommand(new Subsystem() {});
        AutoShootEvents events = new AutoShootEvents(shot);
        Command selected = Commands.idle();
        Command auto = events.wrapAuto(selected);
        assertSame(auto, events.wrapAuto(selected));
        for (int run = 0; run < 2; run++) {
            auto.initialize();
            auto.execute();
            assertEquals(run, shot.starts);
            events.requestStart();
            auto.execute();
            auto.end(true);
        }
        assertEquals(2, shot.starts);
        assertEquals(2, shot.stops);
    }

    private static final class TrackingCommand extends Command {
        int starts;
        int updates;
        int stops;

        TrackingCommand(Subsystem subsystem) {
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            starts++;
        }

        @Override
        public void execute() {
            updates++;
        }

        @Override
        public void end(boolean interrupted) {
            stops++;
        }
    }
}
