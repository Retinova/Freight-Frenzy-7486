package org.firstinspires.ftc.teamcode.supers;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

public class BetterGamepad extends Gamepad {
    public Gamepad prevState;

    @Override
    public void fromByteArray(byte[] byteArray) throws RobotCoreException {
        prevState.copy(this);
        super.fromByteArray(byteArray);
    }

    public void update() throws RobotCoreException{
        prevState.copy(this);
    }
}
