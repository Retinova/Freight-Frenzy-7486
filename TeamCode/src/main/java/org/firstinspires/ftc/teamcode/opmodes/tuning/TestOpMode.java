package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.teamcode.supers.GamepadState;

@Disabled
@TeleOp(name = "BetterTest")
public class TestOpMode extends LinearOpMode {
    GamepadState prevState;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelem = dashboard.getTelemetry();

    ElapsedTime timer = new ElapsedTime();

    AndroidTextToSpeech tts = new AndroidTextToSpeech();

    @Override
    public void runOpMode() throws InterruptedException {
        prevState = new GamepadState(gamepad1);
        tts.initialize();

        timer.reset();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a && !prevState.a) tts.speak("key press");
            prevState.copyState(gamepad1);

            dashTelem.addData("val:", Math.sin(timer.seconds()));
            dashTelem.update();
        }
    }
}

// you are bad