package org.firstinspires.ftc.teamcode.vision;

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
import java.io.File;
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

        cnn = Dnn.readNetFromTensorflow(pb);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        blob = Dnn.blobFromImage(input, 1.0, new Size(input.height(), input.height()), new Scalar(0, 0, 0), true, true, CV_8U);
        cnn.setInput(blob);
        output = cnn.forward();

        return input;
    }

    @Override
    public void onViewportTapped(){

    }
}
