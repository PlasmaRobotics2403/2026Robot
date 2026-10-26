package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;

public final class AutoShootEvents {
    private final Command shot;
    private boolean active;
    private boolean requested;
    private boolean running;
    private int foregroundCommands;
    private final Map<Command, Command> wrappedAutos = new IdentityHashMap<>();

    public AutoShootEvents(Command shot) {
        this.shot = shot;
    }

    public void requestStart() {
        if (active) requested = true;
    }

    public void requestStop() {
        requested = false;
        if (running) {
            running = false;
            shot.end(true);
        }
    }

    public Command wrapNamedCommand(Command command) {
        if (Collections.disjoint(command.getRequirements(), shot.getRequirements())) return command;
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                requestStop();
                foregroundCommands++;
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                try {
                    super.end(interrupted);
                } finally {
                    foregroundCommands--;
                }
            }
        };
    }

    public Command wrapAuto(Command auto) {
        if (wrappedAutos.containsKey(auto)) return wrappedAutos.get(auto);
        Set<Subsystem> requirements = new HashSet<>(auto.getRequirements());
        requirements.addAll(shot.getRequirements());
        Set<Subsystem> reservedRequirements = Set.copyOf(requirements);
        Command wrapped = new WrapperCommand(auto) {
            @Override
            public Set<Subsystem> getRequirements() {
                return reservedRequirements;
            }

            @Override
            public void initialize() {
                requestStop();
                foregroundCommands = 0;
                active = true;
                super.initialize();
            }

            @Override
            public void execute() {
                super.execute();
                if (!requested || foregroundCommands > 0) return;
                if (!running) {
                    running = true;
                    shot.initialize();
                }
                shot.execute();
                if (shot.isFinished()) {
                    running = false;
                    requested = false;
                    shot.end(false);
                }
            }

            @Override
            public void end(boolean interrupted) {
                active = false;
                try {
                    super.end(interrupted);
                } finally {
                    requestStop();
                    foregroundCommands = 0;
                }
            }
        };
        wrappedAutos.put(auto, wrapped);
        return wrapped;
    }
}
