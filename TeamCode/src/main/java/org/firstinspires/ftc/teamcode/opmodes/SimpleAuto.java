package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.supers.Direction;
import org.firstinspires.ftc.teamcode.supers.Mode;
import org.firstinspires.ftc.teamcode.supers.Robot;

@Autonomous(name="simple auto")
public class SimpleAuto extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(this, Mode.AUTO);

        r.initCheck();

        waitForStart();

        r.odo.drive(Direction.FORWARD, 20, 1.0);
        sleep(500);
//        r.odo.turn(90);
//        r.odo.turn(-90);
//        sleep(500);
//        r.odo.drive(Direction.FORWARD, 54, 1.0);
    }
}
