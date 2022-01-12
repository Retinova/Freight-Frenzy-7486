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
//    TelemetryPacket packet = new TelemetryPacket();

    ElapsedTime timer = new ElapsedTime();

    // Toggle variables
    boolean wheelToggle = false, carouselToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware and gamepad state
        r = new Robot(this, Mode.TELEOP);
        prevState = new GamepadState(gamepad1);
//        packet.fieldOverlay().setFill("blue").fillRect(20,20,100,100);

        r.initCheck();

        waitForStart();

        timer.reset();

        while(isStarted() && !isStopRequested()){
            // Handles driving controls
            mecanumDrive();

            // Compliance wheel servo controls
            if (gamepad1.dpad_up && !prevState.dpad_up) {
                r.lwheelrot.setPosition(1);
                r.rwheelrot.setPosition(0);
            }
            if (gamepad1.dpad_down && !prevState.dpad_down) {
                r.lwheelrot.setPosition(0);
                r.rwheelrot.setPosition(0);
            }

            // Compliance wheel controls
            if (gamepad1.x && !prevState.x) {
                wheelToggle = !wheelToggle;

                if (wheelToggle) {
                    r.lwheel.setPower(1);
                    r.rwheel.setPower(1);
                }
                else {
                    r.lwheel.setPower(0);
                    r.rwheel.setPower(0);
                }
            }

            // Carousel motor controls
            if (gamepad1.a && !prevState.a){
                carouselToggle = !carouselToggle;

                if(carouselToggle) r.carousel.setPower(0.5);
                else r.carousel.setPower(0.0);
            }

            // Update gamepad state for next cycle
            prevState.copyState(gamepad1);

            // Telemetry
            telemetry.addData("Gamepad values", "(%.1f, %.1f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    public void mecanumDrive(){
        // Signs determined by wheel vector orientation
        double lfp, lbp, rfp, rbp;
        rfp = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        rbp = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        lfp = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        lbp = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;

        // Sorts to find power of highest magnitude
        double[] powers = {Math.abs(lfp), Math.abs(lbp), Math.abs(rfp), Math.abs(rbp), 1.0};
        Arrays.sort(powers);

        // Divide by the maximum to maintain ratios between wheel powers that would normally get
        // ruined by clipping between [-1.0, 1.0]
        r.lf.setPower(lfp / powers[4]);
        r.lb.setPower(lbp / powers[4]);
        r.rf.setPower(rfp / powers[4]);
        r.rb.setPower(rbp / powers[4]);
    }
}

// Sussy Baka