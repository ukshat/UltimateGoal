package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetermination extends LinearOpMode {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    OpenCvCamera webcam;

    public void initialize() {
        VideoCapture capture = new VideoCapture(1);
        capture.open(0);

        Mat image = new Mat();
        capture.read(image);

    }

    private int ringCount;

    public int countRings(Mat input) {
        initialize();

        return ringCount;
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            sleep(100);
        }

    }

}
