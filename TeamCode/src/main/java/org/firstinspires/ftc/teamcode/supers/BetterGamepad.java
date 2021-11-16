package org.firstinspires.ftc.teamcode.supers;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

@Deprecated
public class BetterGamepad extends Gamepad {
    public GamepadState prevState;

    public static BetterGamepad create(Gamepad gamepad) {
        BetterGamepad better = (BetterGamepad) gamepad;

        better.prevState = new GamepadState(gamepad);

        return better;
    }

    @Override
    public void fromByteArray(byte[] byteArray) throws RobotCoreException {
        prevState.copyState(this);
        super.fromByteArray(byteArray);
    }

    public void update() throws RobotCoreException{
        prevState.copyState(this);
    }
}
