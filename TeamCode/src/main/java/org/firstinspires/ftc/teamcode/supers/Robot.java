package org.firstinspires.ftc.teamcode.supers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.Odometry;

public class Robot {
    public final DcMotor lf, rf, lb, rb, lwheel, rwheel, carousel, arm;
    public final Servo lwheelrot, rwheelrot;
    public final BNO055IMU imu;
    public final BNO055IMU.Parameters params = new BNO055IMU.Parameters();

    public Odometry odo;
    public TeleOp teleOp;

    public LinearOpMode opMode;
    private HardwareMap hwMap;

    public Robot(LinearOpMode opMode, Mode mode) {
        this.opMode = opMode;
        hwMap = opMode.hardwareMap;

        lf = hwMap.get(DcMotor.class, "lf");
        rf = hwMap.get(DcMotor.class, "rf");
        lb = hwMap.get(DcMotor.class, "lb");
        rb = hwMap.get(DcMotor.class, "rb");
        lwheel = hwMap.get(DcMotor.class, "lwheel");
        rwheel = hwMap.get(DcMotor.class, "rwheel");
        carousel = hwMap.get(DcMotor.class, "carousel");
        arm = hwMap.get(DcMotor.class, "arm");

        lwheelrot = hwMap.get(Servo.class, "lwheelrot");
        rwheelrot = hwMap.get(Servo.class, "rwheelrot");

        imu = hwMap.get(BNO055IMU.class, "imu");
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.calibrationDataFile = "BNO055IMUCalibration.json";

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lwheelrot.setDirection(Servo.Direction.REVERSE);

        switch (mode) {
            case TELEOP:
                teleOp = new TeleOp(opMode);
                break;

            case AUTO:
                odo = new Odometry(this);
                break;
        }
    }

    public void initCheck(){
        opMode.telemetry.addData(">", "Initialized");
        opMode.telemetry.update();
    }
}
