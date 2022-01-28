package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.Objects;

/**
 * Class to make sequences of buttons presses do stuff.
 */
public class Konami {

    private static Konami instance = null;

    /**
     * Gets the current instance of Konami.
     *
     * @return The Konami instance
     */
    public static Konami getInstance() {
        if (instance == null) {
            instance = new Konami();
        }

        return instance;
    }

    /**
     * STUFF THAT OWEN DID I HAVE NO IDEA WHAT THIS DOES.
     *
     * @param testString OWEN DID THIS IDK
     * @return OWEN DID THIS IDK
     */
    public static Command registeredButtonPress(String testString) {
        return new CommandBase() {
            @Override
            public void end(boolean interrupted) {
                getInstance().tryStringOnSequences(testString);
            }
        };
    }

    /**
     * STUFF THAT OWEN DID I HAVE NO IDEA WHAT THIS DOES.
     *
     * @return OWEN DID THIS IDK
     */
    public static Command nonRegisteredButtonPress() {
        return new CommandBase() {
            @Override
            public void end(boolean interrupted) {
                getInstance().failAllSequences();
            }
        };
    }

    private final ArrayList<Sequence> sequences;

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

    private static class Sequence {
        private final String[] code;
        private final Command command;

        private int count = 0;

        public Sequence(String[] code, Command command) {
            this.code = code;
            this.command = command;
        }

        public void tryString(String testString) {
            if (Objects.equals(code[count], testString)) {
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
