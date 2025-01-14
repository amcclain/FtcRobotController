package org.thewaverlyschool;

import com.qualcomm.robotcore.hardware.Gamepad;

public class WaverlyGamepad {

    private final Gamepad gp;

    // Latest button press state:
    //  - x = button is currently down.
    //  - xPressed = button was just pressed down for the first time.
    //  - xWasPressed = button was pressed down the last time the loop ran -
    //                  if it is still down, don't count it as a new press
    public boolean a = false;
    public boolean aPressed = false;
    private boolean aWasPressed = false;

    public boolean dpadUp = false;
    public boolean dpadUpPressed = false;
    private boolean dpadUpWasPressed = false;

    public boolean dpadDown = false;
    public boolean dpadDownPressed = false;
    private boolean dpadDownWasPressed = false;

    public boolean dpadLeft = false;
    public boolean dpadLeftPressed = false;
    private boolean dpadLeftWasPressed = false;

    public boolean dpadRight = false;
    public boolean dpadRightPressed = false;
    private boolean dpadRightWasPressed = false;

    public boolean rightBumper = false;
    public boolean rightBumperPressed = false;
    private boolean rightBumperWasPressed = false;

    public boolean leftBumper = false;
    public boolean leftBumperPressed = false;
    private boolean leftBumperWasPressed = false;

    public boolean rightTrigger = false;
    public boolean rightTriggerPressed = false;
    private boolean  rightTriggerWasPressed = false;

    public boolean leftTrigger = false;
    public boolean leftTriggerPressed = false;
    private boolean  leftTriggerWasPressed = false;

    public WaverlyGamepad(Gamepad gamepad) {
        gp = gamepad;
    }

    public void readButtons() {
        a = gp.a;
        aPressed = a && !aWasPressed;
        aWasPressed = a;

        dpadUp = gp.dpad_up;
        dpadUpPressed = dpadUp && !dpadUpWasPressed;
        dpadUpWasPressed = dpadUp;

        dpadDown = gp.dpad_down;
        dpadDownPressed = dpadDown && !dpadDownWasPressed;
        dpadDownWasPressed = dpadDown;

        dpadLeft = gp.dpad_left;
        dpadLeftPressed = dpadLeft && !dpadLeftWasPressed;
        dpadLeftWasPressed = dpadLeft;

        dpadRight = gp.dpad_right;
        dpadRightPressed = dpadRight && !dpadRightWasPressed;
        dpadRightWasPressed = dpadRight;

        leftBumper = gp.left_bumper;
        leftBumperPressed = leftBumper && !leftBumperWasPressed;
        leftBumperWasPressed = leftBumper;

        rightBumper = gp.right_bumper;
        rightBumperPressed = rightBumper && !rightBumperWasPressed;
        rightBumperWasPressed = rightBumper;

        leftTrigger = gp.left_trigger != 0;
        leftTriggerPressed = leftTrigger && !leftTriggerWasPressed;
        leftTriggerWasPressed = leftTrigger;

        rightTrigger = gp.right_trigger != 0;
        rightTriggerPressed = rightTrigger && !rightTriggerWasPressed;
        rightTriggerWasPressed = rightTrigger;
    }

}
