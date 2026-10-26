# Distance-based autonomous shooting

No paths or autos were edited for this integration. Static Shot keeps its existing preset behavior.

## Path Event Name markers

The following exact event names are bound in RobotContainer:

- Shoot or Start Auto Shooting: request distance-based shooting.
- Stop Auto Shooting or Stop Shooter: stop event shooting.

These are name-only EventTrigger bindings; a path marker does not need a named-command child for this route. A point marker starts shooting until a stop event or the end/cancellation of the entire auto. Shooting can continue across path boundaries. Use a stop event where shooting should end; no zoned whileTrue binding is installed.

The event triggers send requests without taking mechanism requirements. AutoShootEvents runs the event shot within the active auto lifecycle, and the auto wrapper reserves its shooter/indexer/turret requirements. This avoids a separately scheduled shooter command interrupting an auto that already owns those mechanisms. Event requests outside an active auto are ignored.

Explicit registered mechanism commands take priority. Their initialization stops event shooting; event shooting requested while they are active waits until they finish. This includes Deploy Intake because it owns the indexer. Intake Only does not conflict. Auto cancellation/completion and mode changes clean up event shooting.

## Named commands in an auto

Shoot and Start Auto Shooting remain registered named commands using the same distance-based shooting class. Stop Auto Shooting and Stop Shooter remain registered stop commands. Named commands can also be used directly in a path marker command tree; these run with that path and end at its completion.

A bare sequential Shoot step runs until interrupted. For a finite stationary step, use a Deadline group with Wait as its first child and Shoot as its second child. Choose the wait duration in the GUI. A later sequential stop cannot be reached while an unlimited Shoot step is still running.

## Shot behavior

The distance-based shot and turret aim update every cycle as the robot moves. The existing0.5-second feed delay is preserved and restarted each activation. Flywheel readiness, shot validity and turret unwinding are logged, but feeding still uses only elapsed time. No new speed gate was imposed. Start/end stop both feeders; ending also stops the flywheel and restores the existing hood default.

Desktop tests exercise event lifetime, stopping/restarting, named-command priority and repeat auto runs with simulated commands. Real shooter accuracy and feeding require robot validation. Build does not deploy.

References: [PathPlanner triggers](https://pathplanner.dev/pplib-triggers.html), [named commands](https://pathplanner.dev/pplib-named-commands.html).
