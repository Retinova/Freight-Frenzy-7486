package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.CNNPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@TeleOp(name = "PipelineTester")
public class PipelineTester extends LinearOpMode {

    OpenCvInternalCamera2 cam;
    CNNPipeline cnn = new CNNPipeline();
    double[] outs = new double[3];

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.setPipeline(cnn);

                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Failed to open camera: ", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (isStarted() && !isStopRequested()){
            outs[0] = cnn.output.get(0, 0)[0];
            outs[1] = cnn.output.get(0, 1)[0];
            outs[2] = cnn.output.get(0, 2)[0];

            telemetry.addData("CNN Amogus:", "\n%.4f\n%.4f\n%.4f", outs[0], outs[1], outs[2]);
            telemetry.addData("CNN out shape:", cnn.output.size());
            telemetry.update();
            sleep(100);
        }

    }
}
