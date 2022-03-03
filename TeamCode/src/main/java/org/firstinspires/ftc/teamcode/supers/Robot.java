package org.firstinspires.ftc.teamcode.supers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

import java.util.List;

public class Robot {
    public final DcMotor lf, rf, lb, rb, carousel, arm;
    public final Servo dropper, claw;
    public final CRServo gate;
    public final BNO055IMU imu;
    public final BNO055IMU.Parameters params = new BNO055IMU.Parameters();

    private final List<LynxModule> hubs;

    public Odometry odo;
    public TeleOp teleOp;

    public LinearOpMode opMode;
    private HardwareMap hwMap;


    public FtcDashboard dash;
    public Telemetry dashT;

    public Robot(LinearOpMode opMode, Mode mode) {
        this.opMode = opMode;
        hwMap = opMode.hardwareMap;

        lf = hwMap.get(DcMotor.class, "lf");
        rf = hwMap.get(DcMotor.class, "rf");
        lb = hwMap.get(DcMotor.class, "lb");
        rb = hwMap.get(DcMotor.class, "rb");
        carousel = hwMap.get(DcMotor.class, "carousel");
        arm = hwMap.get(DcMotor.class, "arm");

        dropper = hwMap.get(Servo.class, "dropper");
        claw = hwMap.get(Servo.class, "claw");

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

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        switch (mode) {
            case TELEOP:
                teleOp = new TeleOp(opMode);
                break;

            case AUTO:
                odo = new Odometry(this);
                initGyro();
                break;
        }

        dash = FtcDashboard.getInstance();
        dashT = dash.getTelemetry();
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

    public void initGyro(){
        imu.initialize(params);
        opMode.telemetry.addData(">", "Gyro Initialized: %s", imu.getSystemStatus());
    }
}
