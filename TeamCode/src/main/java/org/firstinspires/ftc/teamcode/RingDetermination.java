package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;

public class RingDetermination extends LinearOpMode {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    OpenCvCamera webcam;
    private Bitmap image;

    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new UselessGreenBoxDrawingPipeline());
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public static Bitmap Mat2BufferedImage(Mat mat) {
        //Encoding the image
        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".jpg", mat, matOfByte);
        //Storing the encoded Mat in a byte array
        byte[] byteArray = matOfByte.toArray();
        //Preparing the Bitmap Image
        final Bitmap image = BitmapFactory.decodeByteArray(byteArray,0,byteArray.length);
        return image;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Image capture", image);
            telemetry.update();

            sleep(100);
        }

    }

    public Bitmap getImage() {
        return image;
    }

    class UselessGreenBoxDrawingPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            image = Mat2BufferedImage(input);

            return input;
        }
    }

}
