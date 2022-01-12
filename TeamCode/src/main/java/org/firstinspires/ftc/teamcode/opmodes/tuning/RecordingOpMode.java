package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.supers.GamepadState;
import org.firstinspires.ftc.teamcode.supers.Robot;
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
        dtelemetry.addData("Successfully opened camera", "");
        dtelemetry.update();

//        File[] externalFilesDirs = AppUtil.getDefContext().getExternalFilesDirs(null);
        /*for (File file : externalCacheDirs) {
            if (Environment.isExternalStorageRemovable(file)) {
                // Path is in format /storage.../Android....
                // Get everything before /Android
                path = file.getPath().split("/Android")[0];
                break;
            }
        }*/
//        path = externalFilesDirs[1].getPath().split("Android/")[0];

        File path = Environment.getExternalStorageDirectory();


//        File root = new File(path.getPath() + "/DCIM");
        File root = new File("/sdcard/");
        RobotLog.e("External storage path: " + root);
        RobotLog.e("External storage path exists: %s|writable: %s", root.exists(), root.canWrite());
//        RobotLog.e("Sd card readable: %s|writable: %s", sd.exists(), sd.canWrite());
//        String dir = "";
//        for(File file : root.listFiles()){
//            dir += file.getName() + "|";
//        }
//        RobotLog.e("Directory listing: %s", dir);
        // TODO: Get rid of this after first run
        File dirs = new File(root.getPath() + "/left/");
        boolean dir1 = dirs.mkdir();
        dirs = new File(root.getPath() + "/center/");
        boolean dir2 = dirs.mkdir();
        dirs = new File(root.getPath() + "/right/");
        boolean dir3 = dirs.mkdir();

        RobotLog.e("Directory status: %s|%s|%s", dir1, dir2, dir3);

        waitForStart();

        while(isStarted() && !isStopRequested()){
            if((gamepad1.a && !prevState.a) || (gamepad1.x && !prevState.x) || (gamepad1.b && !prevState.b)) {
                recording = !recording;

                if (recording) {
                    SimpleDateFormat sdf = new SimpleDateFormat("MM_dd_yyyy-HH_mm", Locale.getDefault());
                    String currentDateandTime = sdf.format(new Date());

                    String savePath;
                    if (gamepad1.a && !prevState.a) savePath = "/center/";
                    else if (gamepad1.x && !prevState.x) savePath = "/left/";
                    else savePath = "/right/";

                    cam.startRecordingPipeline(
                            new PipelineRecordingParameters.Builder()
                                    .setBitrate(1, PipelineRecordingParameters.BitrateUnits.Mbps)
                                    .setEncoder(PipelineRecordingParameters.Encoder.H264)
                                    .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                                    .setFrameRate(30)
                                    .setPath(root.getPath() + savePath + currentDateandTime + ".mp4")
                                    .build()
                    );

                    RobotLog.e("Began recording to: %s", root.getPath() + savePath + currentDateandTime + ".mp4");
                }
                else{
                    cam.stopRecordingPipeline();
                    RobotLog.e("Stopped recording");
                }
            }
            prevState.copyState(gamepad1);

//            dtelemetry.addData("Frame Count", cam.getFrameCount());
            dtelemetry.addData("FPS", "%.2f", cam.getFps());
//            dtelemetry.addData("Total frame time ms", cam.getTotalFrameTimeMs());
//            dtelemetry.addData("Pipeline time ms", cam.getPipelineTimeMs());
//            dtelemetry.addData("Overhead time ms", cam.getOverheadTimeMs());
//            dtelemetry.addData("Theoretical max FPS", cam.getCurrentPipelineMaxFps());
            dtelemetry.addData("Recording", recording);
            dtelemetry.addData("Gamepad", "%s|%s", gamepad1.a, prevState.a);
            dtelemetry.update();

//            telemetry.update();

        }
    }

    public class RecordingPipeline extends OpenCvPipeline{

        @Override
        public Mat processFrame(Mat input) {
            return input;

        }
    }
}
