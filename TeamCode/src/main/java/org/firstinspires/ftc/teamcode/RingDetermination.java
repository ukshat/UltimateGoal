package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetermination extends LinearOpMode {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    OpenCvCamera webcam;
    private Bitmap image;
    private boolean capture = false;

    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public static Bitmap Mat2Bitmap(Mat mat) {
        //Encoding the image
        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".jpg", mat, matOfByte);
        //Storing the encoded Mat in a byte array
        byte[] byteArray = matOfByte.toArray();
        //Preparing the Bitmap Image
        return BitmapFactory.decodeByteArray(byteArray,0,byteArray.length);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        //move to rings

        capture = true;

        do {
            if (image == null) sleep(100);
            else {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
                break;
            }
        }while (opModeIsActive() && image != null);

        //image recog
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            if(capture){
                image = Mat2Bitmap(input);
                capture = false;
            }
            return input;
        }

        @Override
        public void onViewportTapped() {}
    }

}
