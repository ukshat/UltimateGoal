package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Autonomous")
public class TEST_IMAGE_RECOG extends LinearOpMode {

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

    RevColorSensorV3 color;

    OpenCvCamera webcam;
    private Bitmap image;
    private boolean capture = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // init gyro
        imu = (BNO055IMU) hardwareMap.get("imu");

        // init motors
        motors[0] = (DcMotorEx) hardwareMap.dcMotor.get("LeftFront");
        motors[1] = (DcMotorEx) hardwareMap.dcMotor.get("RightFront");
        motors[2] = (DcMotorEx) hardwareMap.dcMotor.get("LeftRear");
        motors[3] = (DcMotorEx) hardwareMap.dcMotor.get("RightRear");

        color = (RevColorSensorV3) hardwareMap.get("ColorSensorLeft");

        // init zero power behavior
        for (int i = 0; i < 4 && opModeIsActive(); i++) motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // init gyro parameters
        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        initCam();

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


        //back away from stack -- shift to right
        move(3, TILE_LENGTH * 0.5, 0.5);

        sleep(100);

        //Move to stack
        move(0, TILE_LENGTH * 1.5, 0.5);

        //image recognition
        capture = true;

        do {
            if (image == null) sleep(100);
            else {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
                break;
            }
        }while (opModeIsActive());

        int rings = readStack();

        sleep(500);

        println("RINGS", rings);

    }

    int readStack(){
        Bitmap bitmap = image;

        //bitmap to color
        int w = bitmap.getWidth();
        int h = bitmap.getHeight();

        ColorObj[][] img = new ColorObj[w][h];

        for(int x = 0; x < w; x += 4) for(int y = 0; y < h; y += 4) img[x][y] = new ColorObj(bitmap.getPixel(x, y));

        img = filter1(img);

//        img = filter2(img);



        return 4;
    }

//    public static ColorObj[][] filter2(ColorObj[][] cols){
//        ColorObj[][] returnArr = new ColorObj[cols.length][cols[0].length];
//
//        int range = 20;
//
//        for(int x = 0; x < returnArr.length; x++) {
//            for(int y = 0; y < returnArr[0].length; y++) {
//                if(y >= range * 1.1 && cols[x][y-range].getRed() == 0 && cols[x][y-range/2].getRed() == 0 ||
//                   y <= returnArr.length - range * 1.1 && cols[x][y+range].getRed() == 0 && cols[x][y+range/2].getRed() == 0 ||
//                   x <= returnArr.length - range * 1.1 && cols[y+range][x].getRed() == 0 && cols[y+range/2][x].getRed() == 0 ||
//                   x >= range * 1.1 && cols[y-range][x].getRed() == 0 && cols[y-range/2][x].getRed() == 0)
//                    returnArr[x][y] = new Color(0, 0, 255);
//                else returnArr[x][y] = cols[x][y];
//            }
//        }
//
//        cols = returnArr;
//
//        range = 20;
//        for(int y = range; y < height * scaler - range-1; y++) {
//            for(int x = range; x < width * scaler - range-1; x++) {
//                if(cols[x][y-range].getRed() == 255 && cols[x][y+range].getRed() == 255) returnArr[x][y] = new Color(255, 0, 0);
//                else if(cols[x][y-range].getRed() != 255 && cols[x][y+range].getRed() != 255) returnArr[x][y] = new Color(0, 0, 255);
//            }
//        }
//
//        range = 1;
//        for(int i = 0; i < 100; i++) {
//            cols = returnArr;
//            for(int y = range; y < height * scaler - range-1; y++) {
//                for(int x = range; x < width * scaler - range-1; x++) {
//                    if(cols[x][y-range].getRed() == 255 && cols[y-range][x+range].getRed() == 255) returnArr[x][y] = new Color(255, 0, 0);
//                }
//            }
//        }
//
//        range = 1;
//        for(int i = 0; i < 100; i++) {
//            cols = returnArr;
//            for(int y = range; y < height * scaler - range-1; y++) {
//                for(int x = range; x < width * scaler - range-1; x++) {
//                    if(cols[y-range][x].getRed() == 255 && cols[y+range][x+range].getRed() == 255) returnArr[x][y] = new Color(255, 0, 0);
//                }
//            }
//        }
//
//        return returnArr;
//    }

    ColorObj[][] filter1(ColorObj[][] cols){
        //targetColor for ring
        final ColorObj targetCol = new ColorObj(206, 126, 1);

        ColorObj[][] returnArr = new ColorObj[cols.length][cols[0].length];

        for(int x = 0; x < returnArr.length; x++) {
            for(int y = 0; y < returnArr[0].length; y++) {
                int diff = (Math.abs(cols[x][y].getRed() - targetCol.getRed()) + Math.abs(cols[x][y].getBlue() - targetCol.getBlue()) + Math.abs(cols[x][y].getGreen() - targetCol.getGreen()));
                if(diff < 220 && cols[x][y].getRed() > cols[x][y].getBlue() && cols[x][y].getRed() > cols[x][y].getGreen()) returnArr[x][y] = new ColorObj(255, 0, 0);
                else if(cols[x][y].getGreen() > cols[x][y].getBlue()) returnArr[x][y] = new ColorObj(0, 255, 0);
                else returnArr[x][y] = new ColorObj(0, 0, 255);
            }
        }

        return returnArr;

    }

    class ColorObj{
        int r, g, b;

        ColorObj(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }

        ColorObj(int i){
            r = Color.red(i);
            g = Color.green(i);
            b = Color.blue(i);
        }

        int getRed(){return r;}
        int getBlue(){return b;}
        int getGreen(){return g;}

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

    public void initCam() {
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


























    static void dropGoal(){}

    static void launch(){}

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
     * rotates the robot a certain amount of degrees at a given speed
     *
     * @params degrees amount in degrees to rotate the robot (must be between -180 and 180)
     * @params power speed at which motors will run -- affects speed of rotation
     * */
    void rotate(double degrees){

        // Create the object used to keep track of the current angle of the robot
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // The angle at which the robot starts at
        double angle = orientation.firstAngle;

        // The amount we want to turn
        double firstDegrees = degrees;

        // This is our target position
        degrees += currOrientation;

        // The orientation where the robot starts
        final double startAngle = angle;

        // Reset the motors to run using encoders
        for(int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // If we are turning clockwise

        if (firstDegrees < 0){
            setDirection(4);
            // While we are not at the target position
            // Multily by 0.925 to account for jerk
            while (angle > degrees * 0.925 && opModeIsActive()){
                // Updating the object that keeps track of orientation
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // Updates the variable which stores the current direction of the robot
                angle = orientation.firstAngle;
                // Sets all motor powers to 0.2
                for(int i = 0; i < 4; i++) motors[i].setPower(0.2);
                // Delay
                sleep(20);
            }

        }

        // If we are turning counterclockwise
        else {
            setDirection(5);
            while (angle < degrees * 0.925){
                // Updating the object that keeps track of orientation
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // Updates the variable which stores the current direction of the robot
                angle = orientation.firstAngle;
                // Sets all motor powers to 0.2
                for(int i = 0; i < 4; i++) motors[i].setPower(0.2);
                // Delay
                sleep(20);
            }

        }
        // Stop the robot
        for (DcMotor motor: motors){
            motor.setPower(0);
        }

        // Update the direction of the robot
        currOrientation = degrees;

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