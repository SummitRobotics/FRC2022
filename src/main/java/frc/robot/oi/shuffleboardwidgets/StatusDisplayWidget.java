/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi.shuffleboardwidgets;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Colors;
import java.util.HashMap;

/**
 * Add your docs here.
 */
public class StatusDisplayWidget extends SubsystemBase {

    private final NetworkTableEntry entry;
    private String message;
    private Color8Bit defaultColor;
    private final HashMap<String, StatusMessage> nameAndMessage;
    private boolean changed;

    /**
     * Constructor.
     *
     * @param entry Takes in a Network Table Entry
     */
    public StatusDisplayWidget(NetworkTableEntry entry) {
        this.entry = entry;
        entry.forceSetString("");

        changed = true;
        nameAndMessage = new HashMap<>();
        defaultColor = Colors.WHITE;
    }

    /**
     * Sets the default color.
     *
     * @param color the color to use when none is provided, defaults to white
     */
    public void setDefaultColor(Color8Bit color) {
        defaultColor = color;
    }

    /**
     * adds a status to display.
     *
     * @param status the message to display. NOTE: status cannot contain any :'s because of the way
     *               data is encoded
     * @param color  the css compatible color string to display it in for example #ff0000 or red are
     *               both valid and would be the same
     * @param name Status name.
     * @param priority priority of the status message.
     */
    public void addStatus(String name, String status, String color, int priority) {
        changed = true;

        if (status.contains(":")) {
            throw new IllegalArgumentException("Status cannot contain :");
        }

        nameAndMessage.put(name, new StatusMessage(priority, status + ":" + color));
    }

    /**
     * adds a status to display.
     *
     * @param status the message to display. NOTE: status cannot contain any :'s because of the way
     *               data is encoded
     * @param color  the color8bit to display it in
     * @param name Status name.
     * @param priority priority of the status message.
     */
    public void addStatus(String name, String status, Color8Bit color, int priority) {
        changed = true;
        addStatus(name, status, hexStringFomColor(color), priority);
    }

    /**
     * adds a status to display.
     *
     * @param status the message to display. NOTE: status cannot contain any :'s because of the way
     *               data is encoded the takes no color and instead uses the color set with {@link
     *               #setDefaultColor(Color8Bit color)}
     * @param name Status name.
     * @param priority priority of the status message.
     */
    public void addStatus(String name, String status, int priority) {
        changed = true;
        addStatus(name, status, hexStringFomColor(defaultColor), priority);
    }

    /**
     * Gets the current status.
     *
     * @return the current status
     */
    public String getStatus() {
        return message;
    }

    public void removeStatus(String name) {
        changed = true;
        nameAndMessage.remove(name);
    }

    /**
     * Formats a Color8Bit into a hex string.
     *
     * @param color the color to be formatted
     * @return the color in hex format (e.g. #ffffff)
     */
    private String hexStringFomColor(Color8Bit color) {
        return String.format("#%02x%02x%02x", color.red, color.green, color.blue);
    }

    // sorts and displays the messages
    @Override
    public void periodic() {
        if (changed) {

            // sorts through all messages and gets the one with the highest priority
            int currentPriority = Integer.MIN_VALUE;
            String message = "";

            for (StatusMessage x : nameAndMessage.values()) {
                if (x.getPriority() > currentPriority) {
                    currentPriority = x.getPriority();
                    message = x.getMessage();
                }
            }

            // displays the message
            entry.setString(message);

            changed = false;
        }
    }

    /**
     * Internal wrapper for status messages.
     * It so that they can be sorted and the one with the highest priority
     */
    private static class StatusMessage {

        private final int priority;
        private final String message;

        /**
         * Makes a new status.
         *
         * @param priority the integer priority of the message.
         *                  A higher number means a higher priority
         * @param message  the string of the message test to display. CAN NOT CONTAIN A ":"
         */
        public StatusMessage(int priority, String message) {
            this.priority = priority;
            this.message = message;
        }

        public String getMessage() {
            return message;
        }

        public int getPriority() {
            return priority;
        }
    }
}
