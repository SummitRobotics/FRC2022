// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;

public class StatefullSequentalCommandGroup extends CommandGroupBase {
    private enum CommandState {
        NotRun, Inited, Done
    }

    private ArrayList<Map.Entry<Command, CommandState>> cmds = new ArrayList<Map.Entry<Command, CommandState>>();

    private int cmdIndex = 0;


    /** Creates a new StatefullSequentalCommandGroup. */
    public StatefullSequentalCommandGroup(Command... commands) {
        addCommands(commands);
        cmdIndex = 0;
    }

    @Override
    public void addCommands(Command... commands) {
        requireUngrouped(commands);

        //registerGroupedCommands(commands);

        for (Command x : commands) {
            m_requirements.addAll(x.getRequirements());
            cmds.add(new AbstractMap.SimpleEntry<Command, CommandState>(x, CommandState.NotRun));
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (cmdIndex >= cmds.size()) {
            return;
        }

        Map.Entry<Command, CommandState> current = cmds.get(cmdIndex);

        Command c = current.getKey();
        CommandState s = current.getValue();

        if (s.equals(CommandState.NotRun)) {
            c.initialize();
            current.setValue(CommandState.Inited);

        } else if (s.equals(CommandState.Inited)) {
            c.execute();
            if (c.isFinished()) {
                current.setValue(CommandState.Done);
                c.end(false);
                cmdIndex++;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (cmdIndex >= cmds.size()) {
            return;
        }

        if (interrupted) {
            cmds.get(cmdIndex).setValue(CommandState.NotRun);
            cmds.get(cmdIndex).getKey().end(true);
        }
    }

    public void resetCommandState() {
        cmdIndex = 0;
        for (Entry x : cmds) {
            x.setValue(CommandState.NotRun);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cmdIndex >= cmds.size();
    }
}

