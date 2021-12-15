package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.supers.GamepadState;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name="Recorder", group="vision")
public class RecordingOpMode extends LinearOpMode {

    OpenCvCamera cam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dtelemetry = dashboard.getTelemetry();

    GamepadState prevState;

    boolean recording = false;

    @Override
    public void runOpMode() throws InterruptedException {
        prevState = new GamepadState(gamepad1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cam.setPipeline(new RecordingPipeline());

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                dtelemetry.addData("Failed to open camera: ", errorCode);
                dtelemetry.update();
            }
        });

        dashboard.startCameraStream(cam, 0);

        File root = Environment.getRootDirectory();
        // TODO: Get rid of this after first run
        File dirs = new File(root.getPath() + "/DCIM/left");
        dirs.mkdir();
        dirs = new File(root.getPath() + "/DCIM/center");
        dirs.mkdir();
        dirs = new File(root.getPath() + "/DCIM/right");
        dirs.mkdir();

        waitForStart();

        while(isStarted() && !isStopRequested()){
            if((gamepad1.a && !prevState.a) || (gamepad1.x && !prevState.x) || (gamepad1.b && !prevState.b)) {
                recording = !recording;

                if (recording) {
                    SimpleDateFormat sdf = new SimpleDateFormat("MM_dd_yyyy-HH_mm", Locale.getDefault());
                    String currentDateandTime = sdf.format(new Date());

                    String savePath;
                    if (gamepad1.a && !prevState.a) savePath = "center/";
                    else if (gamepad1.x && !prevState.x) savePath = "left/";
                    else savePath = "right/";

                    cam.startRecordingPipeline(
                            new PipelineRecordingParameters.Builder()
                                    .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                                    .setEncoder(PipelineRecordingParameters.Encoder.H264)
                                    .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                                    .setFrameRate(30)
                                    .setPath(root.getPath() + "/DCIM/" + savePath + currentDateandTime + ".mp4")
                                    .build()
                    );
                }
                else cam.stopRecordingPipeline();
            }

            dtelemetry.addData("Frame Count", cam.getFrameCount());
            dtelemetry.addData("FPS", "%.2f", cam.getFps());
//            dtelemetry.addData("Total frame time ms", cam.getTotalFrameTimeMs());
//            dtelemetry.addData("Pipeline time ms", cam.getPipelineTimeMs());
//            dtelemetry.addData("Overhead time ms", cam.getOverheadTimeMs());
            dtelemetry.addData("Theoretical max FPS", cam.getCurrentPipelineMaxFps());
            dtelemetry.update();

            prevState.copyState(gamepad1);
        }
    }

    public class RecordingPipeline extends OpenCvPipeline{

        @Override
        public Mat processFrame(Mat input) {
            return input;
        }
    }
}
