package org.firstinspires.ftc.teamcode.vision;

import android.content.res.AssetManager;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import static org.opencv.core.CvType.CV_8U;

public class CNNPipeline extends OpenCvPipeline {
    private File pbFile;
    public String pb;
    private Net cnn;
    private Mat blob = new Mat();
    public Mat output = new Mat();

    public CNNPipeline(){
//        pbFile = new File(AppUtil.getDefContext().getFilesDir(), "v1_1_frozen_graph.pb");
//        try {
//            pb = pbFile.getCanonicalPath();
//        } catch (IOException e) {
//
//        }

        cnn = Dnn.readNetFromTensorflow(getPath("v1_1_frozen_graph.pb"));
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        blob = Dnn.blobFromImage(input, 1.0, new Size(input.height(), input.height()), new Scalar(0, 0, 0), true, true, CV_8U);
        cnn.setInput(blob);
        output = cnn.forward();

        blob.release();

        return input;
    }

    @Override
    public void onViewportTapped(){

    }

    private String getPath(String file) {
        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        BufferedInputStream inputStream = null;
        try {
            // Read data from assets.
            inputStream = new BufferedInputStream(assetManager.open(file));
            byte[] data = new byte[inputStream.available()];
            inputStream.read(data);
            inputStream.close();
            // Create copy file in storage.
            File outFile = new File(AppUtil.getDefContext().getFilesDir(), file);
            FileOutputStream os = new FileOutputStream(outFile);
            os.write(data);
            os.close();
            // Return a path to file which may be read in common way.
            return outFile.getAbsolutePath();
        } catch (IOException ex) {
            RobotLog.i("Failed to load model file");
        }
        return "";
    }
}
