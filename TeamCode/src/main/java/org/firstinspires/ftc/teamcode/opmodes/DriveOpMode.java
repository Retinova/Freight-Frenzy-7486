package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    int armTarget = 0;
    double servoTarget = 0.0;

    // Other variables
    final double armToDegrees = 360.0 / 1992.6;
    final double degreesToServo = 1.0 / 280.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware and gamepad state
        r = new Robot(this, Mode.TELEOP);
        prevState = new GamepadState(gamepad1);
//        packet.fieldOverlay().setFill("blue").fillRect(20,20,100,100);

        r.initCheck();

        armTarget = r.arm.getCurrentPosition();
        r.arm.setTargetPosition(armTarget);
        r.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        timer.reset();

        r.arm.setPower(0.8);

        while(isStarted() && !isStopRequested()){
            // Handles driving controls
            mecanumDrive();

            // Arm controls
            armTarget += (int) (gamepad1.right_trigger*200 - gamepad1.left_trigger*200) / 20;
            r.arm.setTargetPosition(armTarget);

            servoTarget = r.arm.getTargetPosition() * armToDegrees * degreesToServo;
            r.dropper.setPosition(Range.clip(servoTarget, 0.0, 1.0));


            // Carousel motor controls
            if (gamepad1.dpad_left && !prevState.dpad_left){
                carouselToggle = !carouselToggle;

                if(carouselToggle) r.carousel.setPower(1.0);
                else r.carousel.setPower(0.0);
            }
            if (gamepad1.dpad_right && !prevState.dpad_right){
                carouselToggle = !carouselToggle;

                if(carouselToggle) r.carousel.setPower(-1.0);
                else r.carousel.setPower(0.0);
            }

            // Update gamepad state for next cycle
            prevState.copyState(gamepad1);

            // Telemetry
            telemetry.addData("Gamepad LS values", "(%.1f, %.1f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Arm target", "%d", armTarget);
            telemetry.addData("Servo target", "%.4f", servoTarget);
            telemetry.update();

            // Grab new encoder values
            r.bulkRead();
        }
    }

    public void mecanumDrive(){
        // Signs determined by wheel vector orientation
        double lfp, lbp, rfp, rbp;
        rfp = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        rbp = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        lfp = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        lbp = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

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

    public void manageArm(){
        // TODO: measure positions vs. angle (radians, Q1,2,4), or change to PD loop or something if needed
        // https://www.desmos.com/calculator/wgxtt5cy8x
        double angle = r.arm.getCurrentPosition() / 1.0 * 1.0;
        double pwrProp = (80.7853310966) * Math.cos(angle) / 93.6;
        r.arm.setPower(pwrProp);
    }
}

// Sussy Baka (aka Ron)