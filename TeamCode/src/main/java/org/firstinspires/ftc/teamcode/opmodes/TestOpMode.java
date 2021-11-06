package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.supers.BetterGamepad;

public class TestOpMode extends LinearOpMode {
    BetterGamepad gamepad1 = (BetterGamepad) super.gamepad1;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Keypress: ", gamepad1.a && !gamepad1.prevState.a);
        /*try{
            gamepad1.update();
        } catch(RobotCoreException e){
            telemetry.addData(">", "Failed to update gamepad:\n%s", e.getStackTrace());
        }*/
    }
}
