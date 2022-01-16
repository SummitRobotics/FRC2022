package frc.robot.devices.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDCall implements LEDHandler {

    private String name;
    private int priority;
    private LEDRange range;

    private Color8Bit defaultColor;

    private int startLoop = 0;


    /**
     * Creates a new LEDCall
     * 
     * @param name the unique name of the call
     * @param priority the priority, where higher values override lower values
     * @param range the LEDRange the call applies to
     */
    public LEDCall(String name, int priority, LEDRange range) {
        this.name = name;
        this.priority = priority;
        this.range = range;

        defaultColor = new Color8Bit(Color.kBlack);

        startLoop = 0;
    }

    /**
     * Creates a new LEDCall without a supplied name
     * 
     * @param priority the priority, where higher values override lower values
     * @param range the LEDRange the call applies to
     */
    public LEDCall(int priority, LEDRange range) {
        this(LEDs.getInstance().getUniqueID(), priority, range);
    }

    /**
     * Gets the priority of the call. Larger numbers have greater priority.
     * 
     * @return the priority
     */
    public int getPriority() {
        return priority;
    }

    /**
     * Gets the used LEDRange
     * 
     * @return the LEDRange
     */
    public LEDRange getRange() {
        return range;
    }

    public String getName() {
        return name;
    }

    public LEDHandler activate() {
        LEDs.getInstance().addCall(name, this);
        return this;
    }

    public void cancel() {
        LEDs.getInstance().removeCall(name);
    }

    /**
     * The color generator function, which can be overloaded. By default it returns black
     * 
     * @param loop the current LED loop
     * @param led the LED color in question
     * @return the color of the LED
     */
    public Color8Bit getColor(int loop, int led) {
        return defaultColor;
    }

    /**
     * Creates a new LEDCall with getColor overloaded. Makes all LEDs in its range one color.
     * 
     * @param color the solid color to use
     * @return the modified LEDCall
     */
    public LEDCall solid(Color8Bit color) {
        return new LEDCall(priority, range) {
            @Override
            public Color8Bit getColor(int loop, int led) {
                return color;
            }
        };
    }

    /**
     * Creates a new LEDCall with getColor overloaded. Makes LEDs in its range flash between 2 colors at a set interval.
     * 
     * @param onColor the first color to flash
     * @param offColor the second color to flash
     * @return the modified LEDCall
     */
    public LEDCall flashing(Color8Bit onColor, Color8Bit offColor) {
        return new LEDCall(priority, range) {
            @Override
            public Color8Bit getColor(int loop, int led) {
                int time = loop % 14;
                if (time < 7) {
                    return onColor;

                } else {
                    return offColor;
                }    
            }
        };
    }

    /**
     * Creates a new LEDCall with getColor overloaded. Makes LEDs in its range flash color ON, then color OFF, the color ON, then color OFF, then hold color ON.
     * 
     * @param onColor color ON
     * @param offColor color OFF
     * @return the modified LEDCall
     */
    public LEDCall ffh(Color8Bit onColor, Color8Bit offColor) {
        return new LEDCall(priority, range) {
            @Override
            public Color8Bit getColor(int loop, int led) {
                if(startLoop == 0){
                    startLoop = loop;
                }
                int time = loop - startLoop;
                if (time <= 8) {
                    return onColor;

                } else if (time <= 16) {
                    return offColor;

                } else if(time <= 24) {
                    return onColor;

                } else if(time <= 32) {
                    return offColor;

                } else {

                    return onColor;
                }
            }
        };
    }

    /**
     * Creates a new LEDCall with getColor overloaded. Makes LEDs in its range do a wave?
     * 
     * @param color the color to do something with?
     * @return the modified LEDCall
     */
    public LEDCall sine(Color8Bit color) {
        return new LEDCall(priority, range) {
            private final double waveLength = 6.068;

            @Override
            public Color8Bit getColor(int loop, int led) {
                double scaler = (Math.cos((((led * Math.PI) / waveLength) - (loop/4)) + 3)+1) * 0.25;
                return new Color8Bit(
                    (int) ((color.red * scaler) + 0.5),
                    (int) ((color.green * scaler) + 0.5),
                    (int) ((color.blue * scaler) +.5)
                );
            }
        };
    }

    /**
     * Creates a new LEDCall that makes a rainbow
     * 
     * @return the modified LEDCall
     */
    public LEDCall rainbow() {
        return new LEDCall(priority, range) {
            @Override
            public Color8Bit getColor(int loop, int led) {
                return new Color8Bit(Color.fromHSV(
                    (loop + led) * 2 % 180, 
                    255, 255));
            }
        };
    }
}