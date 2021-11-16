package org.firstinspires.ftc.teamcode.supers;

import com.qualcomm.robotcore.hardware.Gamepad;

// Class to store gamepad values

public class GamepadState {
    public boolean
            a = false,
            b = false,
            y = false,
            x = false,
            dpad_up = false,
            dpad_down = false,
            dpad_right = false,
            dpad_left = false,
            guide = false,
            start = false,
            back = false,
            left_bumper = false,
            right_bumper = false,
            left_stick_button = false,
            right_stick_button = false;
    public float
            left_stick_x = 0.0f,
            left_stick_y = 0.0f,
            right_stick_x = 0.0f,
            right_stick_y = 0.0f,
            left_trigger = 0.0f,
            right_trigger = 0.0f;

    public GamepadState(Gamepad gamepad){
        copyState(gamepad);
    }


    // updates values from a gamepad
    public void copyState(Gamepad gamepad){
        this.a = gamepad.a;
        this.b = gamepad.b;
        this.y = gamepad.y;
        this.x = gamepad.x;
        this.dpad_up = gamepad.dpad_up;
        this.dpad_down = gamepad.dpad_down;
        this.dpad_left = gamepad.dpad_left;
        this.dpad_right = gamepad.dpad_right;
        this.guide = gamepad.guide;
        this.start = gamepad.start;
        this.back = gamepad.back;
        this.left_bumper = gamepad.left_bumper;
        this.right_bumper = gamepad.right_bumper;
        this.left_stick_button = gamepad.left_stick_button;
        this.right_stick_button = gamepad.right_stick_button;

        this.left_stick_x = gamepad.left_stick_x;
        this.left_stick_y = gamepad.left_stick_y;
        this.right_stick_x = gamepad.right_stick_x;
        this.right_stick_y = gamepad.right_stick_y;
        this.left_trigger = gamepad.left_trigger;
        this.right_trigger = gamepad.right_trigger;
    }
}
