package frc.robot.oi;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Konami {

    private static Konami instance = null;

    public static Konami getInstance() {
        if (instance == null) {
            instance = new Konami();
        }

        return instance;
    }

    public static Command registeredButtonPress(String testString) {
        return new CommandBase(){
            @Override
            public void end(boolean interrupted) {
                getInstance().tryStringOnSequences(testString);
            }
        };
    }

    public static Command nonRegisteredButtonPress() {
        return new CommandBase(){
            @Override
            public void end(boolean interrupted) {
                getInstance().failAllSequences();
            }
        };
    }


    private ArrayList<Sequence> sequences;

    private Konami() {
        sequences = new ArrayList<>();
    }

    private void tryStringOnSequences(String testString) {
        for (Sequence s : sequences) {
            s.tryString(testString);
        }
    }

    private void failAllSequences() {
        for (Sequence s : sequences) {
            s.resetCount();
        }
    }

    public void addSequence(Command command, String... code) {
        sequences.add(new Sequence(code, command));
    }

    
    private class Sequence {
        private String[] code;
        private Command command;

        private int count = 0;

        public Sequence(String[] code, Command command) {
            this.code = code;
            this.command = command;
        }

        public void tryString(String testString) {
            if (code[count] == testString) {
                count++;

                if (count == code.length) {
                    CommandScheduler.getInstance().schedule(command);
                    resetCount();
                }
            
            } else {
                resetCount();
            }
        }

        public void resetCount() {
            count = 0;
        }
    }
}
