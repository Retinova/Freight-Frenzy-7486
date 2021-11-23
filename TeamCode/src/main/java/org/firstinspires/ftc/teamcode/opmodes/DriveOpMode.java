package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.supers.GamepadState;
import org.firstinspires.ftc.teamcode.supers.Mode;
import org.firstinspires.ftc.teamcode.supers.Robot;

import java.util.Arrays;

@TeleOp(name="Drive TeleOp", group="teleop")
public class DriveOpMode extends LinearOpMode {
    Robot r;
    GamepadState prevState;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(this, Mode.TELEOP);
        prevState = new GamepadState(gamepad1);

        r.initCheck();
        waitForStart();

        timer.reset();

        while(isStarted() && !isStopRequested()){
            mecanumDrive();
        }
    }

    public void mecanumDrive(){
        double lfp, lbp, rfp, rbp;
        rfp = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        rbp = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        lfp = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        lbp = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

        double[] powers = {Math.abs(lfp), Math.abs(lbp), Math.abs(rfp), Math.abs(rbp), 1.0};
        Arrays.sort(powers);

        r.lf.setPower(lfp / powers[4]);
        r.lb.setPower(lbp / powers[4]);
        r.rf.setPower(rfp / powers[4]);
        r.rb.setPower(rbp / powers[4]);
    }
}
