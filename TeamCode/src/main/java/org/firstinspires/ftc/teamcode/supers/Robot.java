package org.firstinspires.ftc.teamcode.supers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.Odometry;

import java.util.List;

public class Robot {
    public final DcMotor lf, rf, lb, rb, lwheel, rwheel, carousel, arm;
    public final Servo lwheelrot, rwheelrot, dropper;
    public final CRServo gate;
    public final BNO055IMU imu;
    public final BNO055IMU.Parameters params = new BNO055IMU.Parameters();

    private final List<LynxModule> hubs;

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
        dropper = hwMap.get(Servo.class, "dropper");

        gate = hwMap.get(CRServo.class, "gate");

        imu = hwMap.get(BNO055IMU.class, "imu");
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.calibrationDataFile = "BNO055IMUCalibration.json";

        // Setting all hubs to manual update mode for bulk reads
        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void bulkRead(){
        // Will run one bulk read per cycle,
        // even as frontLeftMotor.getPosition() is called twice
        // because the caches are being handled manually and cleared
        // once a loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

    }
}
