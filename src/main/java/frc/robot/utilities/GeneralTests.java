package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.devices.PCM;
import frc.robot.subsystems.Drivetrain;
import java.util.HashMap;

/**
 * Testable class that includes test logic not belonging to an individual subsystem.
 */
public class GeneralTests implements Testable {
    private PCM pcm;
    private Drivetrain drivetrain;
    private Timer timer;

    // TODO - set these
    private static final double
        FULL_PRESSURE = 0,
        PARTIAL_PRESSURE = 0;

    private boolean hasReachedFullPressure;
    private boolean hasReachedPartialPressure;

    /**
     * Testable class that includes test logic not belonging to an individual subsystem.
     *
     * @param pcm The pneumatics controller
     * @param drivetrain The drivetrain subsystem
     */
    public GeneralTests(Drivetrain drivetrain, PCM pcm) {
        this.pcm = pcm;
        this.drivetrain = drivetrain;
        hasReachedFullPressure = false;
        hasReachedPartialPressure = false;

        timer = new Timer();
    }

    @Override
    public String getTestName() {
        return "General Tests";
    }

    @Override
    public HashMap<String, Boolean> initCustomTests() {
        HashMap<String, Boolean> result = new HashMap<String, Boolean>();
        result.put("Compressor", false);
        result.put("Pressure", false);

        timer.start();

        return result;
    }

    @Override
    public HashMap<String, Boolean> runCustomTests() {
        HashMap<String, Boolean> result = new HashMap<String, Boolean>();
        boolean pressureStatus = false;

        if (hasReachedFullPressure) {
            if (hasReachedPartialPressure) {
                pressureStatus = true;
            } else {
                if (timer.hasElapsed(0.25)) {
                    drivetrain.toggleShift();
                    timer.reset();
                    if (pcm.getPressure() < PARTIAL_PRESSURE) {
                        hasReachedPartialPressure = true;
                    }
                }
            }

        } else if (pcm.getPressure() > FULL_PRESSURE) {
            hasReachedFullPressure = true;
        }

        result.put("Compressor", pcm.getCompressor() && pcm.getCompressorCurrent() > 0);
        result.put("Pressure", pressureStatus);
        
        return result;
    }
}
