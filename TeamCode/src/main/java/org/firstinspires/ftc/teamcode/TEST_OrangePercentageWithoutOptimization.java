package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Autonomous")
public class TEST_OrangePercentageWithoutOptimization extends LinearOpMode {

    // length of a tile
    static final double TILE_LENGTH = 23.5;

    //pid vals
    static final double[] pidfVals = {10, 3, 0, 0};

    // number of ticks in one inch
    static final double TICKS_PER_INCH = 34.2795262044082261656;

    // constant for offset between front/back to left/right
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;

    // stores the current direction of the robot
    double currOrientation;

    DcMotorEx[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotorEx[4];
    // Variables used to initialize gyro
    BNO055IMU imu;
    BNO055IMU.Parameters params;

    ElapsedTime runtime;
    String str = "";

    OpenCvCamera webcam;
    RingCounterPipeline pipeline = new RingCounterPipeline();
    volatile boolean capturing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // init gyro
        imu = (BNO055IMU) hardwareMap.get("imu");

        // init motors
        motors[0] = (DcMotorEx) hardwareMap.dcMotor.get("LeftFront");
        motors[1] = (DcMotorEx) hardwareMap.dcMotor.get("RightFront");
        motors[2] = (DcMotorEx) hardwareMap.dcMotor.get("LeftRear");
        motors[3] = (DcMotorEx) hardwareMap.dcMotor.get("RightRear");

        runtime = new ElapsedTime();

        // init zero power behavior
        for (int i = 0; i < 4 && opModeIsActive(); i++) motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // init gyro parameters
        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        runtime.reset();

        initCam();

        str += runtime.toString() + ", ";

        waitForStart();

        for (int i = 0; i < 4 && opModeIsActive(); i++){
            PIDFCoefficients pidfCoef = motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidfCoef.p = pidfVals[0];
            pidfCoef.i = pidfVals[1];
            pidfCoef.d = pidfVals[2];
            pidfCoef.f = pidfVals[3];
            motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        }

        for(int i = 0; i < 4 && opModeIsActive(); i++) println("" + i, motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        //Move to stack
        move(0, TILE_LENGTH * 1.5 + 6, 0.5);

        sleep(100);

        runtime.reset();

        capturing = true;

        int rings = pipeline.getRingCount();

        str += runtime.toString();

        println("Times", str);

    }

    public void initCam() {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }



    class RingCounterPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        private int ringCount = -1;

        Rect rect = new Rect(
                new Point(320 * 0.25, 240 * 0.05),
                new Point(320 * 0.9, 240 * 0.9)
        );

        public int getRingCount() {
            while (ringCount == -1) {
                sleep(20);
            }
            return ringCount;
        }

        @Override
        public Mat processFrame(Mat input) {
            // enters this if statement if we have reached the rings and are attempting to capture an image
            if(capturing) {
                // immediately set capturing as false so that it exits the while loop and begins driving to the next location while we determine number of rings
                capturing = false;
                // ring detection code

                Mat mat = new Mat();
                Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV_FULL);

                Scalar lowHSV = new Scalar(27, 50, 50);
                Scalar highHSV = new Scalar(47, 255, 255);

                Core.inRange(mat, lowHSV, highHSV, mat);

                // crop the image to remove useless background
                mat = mat.submat(rect);

                double percentOrange = Core.sumElems(mat).val[0] / rect.area() / 255;
                mat.release();

                if (percentOrange < 0.0545) {
                    telemetry.addData("Rings", "ZERO, " + (percentOrange * 100) + " % orange\n");
                    ringCount = 0;
                } else if (percentOrange < 0.185) {
                    telemetry.addData("Rings", "ONE, " + (percentOrange * 100) + " % orange\n");
                    ringCount = 1;
                } else {
                    telemetry.addData("Rings", "FOUR, " + (percentOrange * 100) + " % orange\n");
                    ringCount = 4;
                }
                telemetry.update();

                str += runtime.toString() + ", ";

                runtime.reset();

                webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
                {
                    @Override
                    public void onClose() {
                        webcam.stopStreaming();
                    }
                });
            }
            return input;
        }

        @Override
        public void onViewportTapped() {}

    }

    void move(int config, double distance, double speed){
        setDirection(config);
        for(int i = 0; i < 4 && opModeIsActive(); i++){
            // reset encoders
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // set power to motors
//            motors[i].setPower(speed);
            // set target position
            // if left/right, multiply by horizontal strafe constant
            motors[i].setTargetPosition((int)(distance * TICKS_PER_INCH * (config % 2 == 1 ? HORIZONTAL_STRAFE : 1)));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving(motors, true);

        //stop motors
        for(int i = 0; i < 4 && opModeIsActive(); i++){
            motors[i].setPower(0);
        }
    }

    void moving(DcMotorEx[] motors, boolean slowDown){
        // stores the total distance to move
        final int totalTick = motors[0].getTargetPosition();

        // continue the while loop until all motors complete movement
        while ((motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()) && opModeIsActive()){
            // delay
            try {Thread.sleep(75);} catch (InterruptedException e) {} //sleep
            // if we choose to, increment speed gradually to minimize jerking motion
            if(slowDown){
                int currPos = motors[0].getCurrentPosition();
                double pow = f(currPos, totalTick);
                for(DcMotorEx motor : motors) motor.setVelocity(40 * pow * TICKS_PER_INCH);
            }
        }
    }

    // function to calculate power for motors given distance and current distance to ensure gradual increase and decrease in motor powers
    // an equation for graph of powers assuming that the highest power is 0.5; graph it in Desmos to see
    static double f(int x, int n){
        return -Math.pow((2.6 * Math.pow(x - n / 2, 2)) / (n * n), 1.75) + 0.5;
    }

    /**
     * Configs:
     *  0: forward
     *  1: right
     *  2: backward
     *  3: left
     *  4: clockwise
     *  5: counterclockwise
     * @param config
     */
    void setDirection(int config){

        for(int i = 0; i < 4 && opModeIsActive(); i++){
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            if (config == 0 && i % 2 == 0) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 1 && i >= 2) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 2 && i % 2 == 1) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 3 && i <= 1) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 4) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            }
        }

    }

    void println(String cap, Object val){
        telemetry.addData("\n" + cap, val);
        telemetry.update();
    }
}