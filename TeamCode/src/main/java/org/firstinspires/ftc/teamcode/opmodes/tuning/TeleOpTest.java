package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.teamcode.supers.GamepadState;

@Disabled
@TeleOp(name = "TeleOpTest")
public class TeleOpTest extends LinearOpMode {

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelem = dashboard.getTelemetry();

    ElapsedTime timer = new ElapsedTime();

    AndroidTextToSpeech tts = new AndroidTextToSpeech();

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

        tts.initialize();

        timer.reset();

        waitForStart();

        while(opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(gamepad1.left_stick_y);
            motor3.setPower(gamepad1.right_stick_y);
            motor4.setPower(gamepad1.right_stick_y);



            dashTelem.addData("Left Y >",gamepad1.left_stick_y);
            dashTelem.addData("Right Y >",gamepad1.right_stick_y);
            dashTelem.update();
        }
    }
}

// you are bad