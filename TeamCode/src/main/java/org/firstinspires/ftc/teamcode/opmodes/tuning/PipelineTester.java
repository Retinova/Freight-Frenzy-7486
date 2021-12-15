package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.CNNPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "PipelineTester", group="vision")
public class PipelineTester extends LinearOpMode {

    OpenCvCamera cam;
    CNNPipeline cnn = new CNNPipeline();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.setPipeline(cnn);

                cam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Failed to open camera: ", errorCode);
                telemetry.update();
            }
        });

        dashboard.startCameraStream(cam, 0);

        waitForStart();

        while (isStarted() && !isStopRequested()){
//            outs[0] = cnn.output.get(0, 0)[0];
//            outs[1] = cnn.output.get(0, 1)[0];
//            outs[2] = cnn.output.get(0, 2)[0];

//            telemetry.addData("CNN Amogus:", "\n%.4f\n%.4f\n%.4f", outs[0], outs[1], outs[2]);
//            telemetry.addData("FPS: ", "%.2f", cam.getFps());
//            telemetry.addData("Layer 1: ", cnn.layers.get(0));
//            telemetry.update();

            dashboard.getTelemetry().addData("Left:", "%.4f", cnn.latest[1]);
            dashboard.getTelemetry().addData("Center", "%.4f", cnn.latest[0]);
            dashboard.getTelemetry().addData("Right", "%.4f", cnn.latest[2]);
            dashboard.getTelemetry().addData("FPS: ", "%.2f", cam.getFps());
            dashboard.getTelemetry().update();
//            sleep(100);
        }

        cam.closeCameraDevice();
    }
}