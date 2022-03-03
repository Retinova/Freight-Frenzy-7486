package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.supers.Mode;
import org.firstinspires.ftc.teamcode.supers.Robot;

@TeleOp(group="tuning", name="Encoder test")
public class EncoderTest extends LinearOpMode {
//    Robot r;
    private DcMotor pod;

    public void runOpMode() throws InterruptedException{
//        r = new Robot(this, Mode.AUTO);
        pod = hardwareMap.get(DcMotor.class, "pod");

        waitForStart();

        while(isStarted() && !isStopRequested()){
//            r.bulkRead();

            telemetry.addData("Encoder 1: ", pod.getCurrentPosition());
//            telemetry.addData("Encoder 2: ", r.lf.getCurrentPosition());
//            telemetry.addData("Encoder 3: ", r.lf.getCurrentPosition());
            telemetry.update();
        }
    }
}
