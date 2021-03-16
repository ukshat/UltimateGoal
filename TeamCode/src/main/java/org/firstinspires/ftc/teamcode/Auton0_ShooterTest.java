package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Autonomous")
public class Auton0_ShooterTest extends LinearOpMode {

    // length of a tile
    static final double TILE_LENGTH = 23.5;

    //pid vals
    static final double[] DC_PIDF_VALS = {1.17, 0.117, 0, 11.7};

    // number of ticks in one inch
    static final double TICKS_PER_INCH = 34.2795262044082261656;

    // constant for offset between front/back to left/right
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;

    // stores the current direction of the robot
    double currOrientation, startAngle;

    Orientation orientation;

    volatile Mat img;

    DcMotorEx[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotorEx[4];
    // Variables used to initialize gyro
    BNO055IMU imu;
    BNO055IMU.Parameters params;

    ElapsedTime runTime;

    @Override
    public void runOpMode() throws InterruptedException {
        // init gyro
        imu = (BNO055IMU) hardwareMap.get("imu");

        // init motors
        motors[0] = (DcMotorEx) hardwareMap.dcMotor.get("LeftFront");
        motors[1] = (DcMotorEx) hardwareMap.dcMotor.get("RightFront");
        motors[2] = (DcMotorEx) hardwareMap.dcMotor.get("LeftRear");
        motors[3] = (DcMotorEx) hardwareMap.dcMotor.get("RightRear");

        // init zero power behavior
        for (int i = 0; i < 4 && opModeIsActive(); i++) motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // init gyro parameters
        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);


        WobbleMech wobble = new WobbleMech();

        runTime = new ElapsedTime();


        Shooter shooter = new Shooter();

//        wobble.setPosition(0);
//        sleep(5000);
//        wobble.close();

        waitForStart();

        sleep(1000);

        shooter.setRampState(false);

        sleep(30000);
        /*

        wobble.setPosition(50);

        for (int i = 0; i < 4 && opModeIsActive(); i++){
            PIDFCoefficients pidfCoef = motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidfCoef.p = DC_PIDF_VALS[0];
            pidfCoef.i = DC_PIDF_VALS[1];
            pidfCoef.d = DC_PIDF_VALS[2];
            pidfCoef.f = DC_PIDF_VALS[3];
            motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        }

        //Move to stack
        move(0,     TILE_LENGTH * 1.5 + 6, 0.5);

        sleep(100);

        move(0, TILE_LENGTH - 3, 0.5);


        sleep(100);

        rotate(Math.toDegrees(-Math.atan(0.5/3)) * 2.0/3);


        shooter.shoot();

        */
    }























    class WobbleMech {

        private Servo servo;

        private DcMotorEx motor;
        private AnalogInput inp;
        private volatile double target;
        private volatile boolean shouldMove = true;
        private boolean isClosed;
        final double lowerBound = 0.3, upperBound = 2.0;

        public WobbleMech() {

            servo = hardwareMap.servo.get("wobbleservo");
            servo.setDirection(Servo.Direction.FORWARD);
            motor = (DcMotorEx) hardwareMap.get("wobblemotor");
            inp = hardwareMap.analogInput.get("wobblepot");
            close();
        }

        public boolean isClosed (){

            return isClosed;

        }

        public void close (){

            servo.setPosition(0);
            isClosed = true;
        }

        public void open (){

            servo.setPosition(1);
            isClosed = false;
        }

        public double getPosition() {
            return (inp.getVoltage() - lowerBound) * 100 / (upperBound - lowerBound);
        }

        public void setPosition(double newTarget){

            if (shouldMove) {
                this.target = newTarget;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        shouldMove = false;
                        if (target > getPosition()) {
                            motor.setVelocity(-100);
                            while (!shouldMove && target > getPosition() && opModeIsActive() && inp.getVoltage() < upperBound){
                                sleep(20);
                            }
                        } else {
                            motor.setVelocity(100);
                            while (!shouldMove && target < getPosition() && opModeIsActive() && inp.getVoltage() > lowerBound){
                                sleep(20);
                            }
                        }

                        shouldMove = true;

                        motor.setVelocity(0);
                    }
                }).start();
            }
            else{
                shouldMove = true;
                setPosition(newTarget);
            }
        }
    }

    class Shooter {

        private final DcMotorEx shooter, intake, pushDownMotor;
        private final Servo stick;
        private final AnalogInput rampPot;
        private volatile boolean rampState = false; // true for down & ready to shoot, false for up, and not ready to shoot
        public final static double upperBound = 1.46, lowerBound = 0.5; // 1.46, 0.55

        public Shooter() {
            rampPot = hardwareMap.analogInput.get("shooterpot");
            pushDownMotor = hardwareMap.get(DcMotorEx.class, "ramp");
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            stick = hardwareMap.servo.get("shoot");
            intake = hardwareMap.get(DcMotorEx.class, "intake");

        }

        // true for down & ready to shoot, false for up, and not ready to shoot
        public boolean getRampState(){
            return rampState;
        }

        public void setRampState(boolean newState){
            if(newState) {
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        pushDownMotor.setVelocity(50);

                        while(rampPot.getVoltage() > lowerBound * 1.1) sleep(20);

                        pushDownMotor.setVelocity(0);

                        rampState = true;
                    }
                }).start();
            }else{
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        pushDownMotor.setVelocity(-100);

                        while(rampPot.getVoltage() < upperBound) sleep(20);

                        pushDownMotor.setVelocity(0);

                        rampState = false;
                    }
                }).start();
            }
        }

        public void push() {
            if(rampState) stick.setPosition(1);
        }

        public void pull() {
            if(rampState) stick.setPosition(0);
        }

        public void shoot() {
            if(rampState) {
                shooter.setVelocity(1300);
                push();
                sleep(1500);
                pull();
    //            intake.setVelocity(-1300);
    //            push();
    //            sleep(1500);
    //            pull();
    //            intake.setVelocity(0);
    //            shooter.setVelocity(0);
            }
        }

    }

    double map(double from){
        return from / HORIZONTAL_STRAFE;
    }

    double findLargest(double[] powers){
        double largest = Math.abs(powers[0]);
        for(double d: powers) if(Math.abs(d) > largest) largest = Math.abs(d);
        return largest;
    }

    void move(double deg, double distance, double speed){

        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngle = orientation.firstAngle;

        setDirection(0);
        //inch to ticks
        distance *= TICKS_PER_INCH;

        //deg to rad
        deg = Math.toRadians(deg);

        //x and y of mapped point on ellipse
        double x = Math.cos(deg) * distance, y = map(Math.sin(deg)) * distance;

        //rotate axis about origin by 45 deg ccw
        deg = Math.asin(y/distance) + Math.PI/4;

        //rewrite x and y to be mapped to new axis
        x = Math.cos(deg) * distance;
        y = map(Math.sin(deg)) * distance;

        /*Front Left, Front Right, Back Left, Back Right*/
        for(int i = 0; i < 4 && opModeIsActive(); i++) motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //FL
        motors[0].setTargetPosition((int) y);
        //FR
        motors[1].setTargetPosition((int) x);
        //BL
        motors[2].setTargetPosition((int) x);
        //BR
        motors[3].setTargetPosition((int) y);

        for(int i = 0; i < 4 && opModeIsActive(); i++) motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set x and y to be between -0.5 and 0.5
        double max = Math.max(Math.abs(x), Math.abs(y));
        x = x / max * speed;
        y = y / max * speed;

        while ((motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()) && opModeIsActive()){
            // delay
            sleep(75);

            //current position of motor travel
            int currPosX = motors[1].getCurrentPosition();
            int currPosY = motors[0].getCurrentPosition();

            //calculate ticks per second
            double xPow = fWithMaxPow(currPosX, (int)distance, x) * 40 * TICKS_PER_INCH;
            double yPow = fWithMaxPow(currPosY, (int)distance, y) * 40 * TICKS_PER_INCH;

            //FL
            motors[0].setVelocity((int) yPow);
            //FR
            motors[1].setVelocity((int) xPow);
            //BL
            motors[2].setVelocity((int) xPow);
            //BR
            motors[3].setVelocity((int) yPow);
        }

        for(DcMotorEx m: motors) m.setVelocity(0);

//        setDirection(4);
//
//        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double currAngle = orientation.firstAngle;
//
//        rotate(currAngle - startAngle);
//
////        telemetry.addData("angles", currAngle + ", " + startAngle);
////        telemetry.update();
////        for(int i = 0; i < 4; i++) motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        for(int i = 0; i < 4; i++) motors[i].setPower(0.2);
////
////        while (currAngle < startAngle){
////            // Updating the object that keeps track of orientation
////            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            // Updates the variable which stores the current direction of the robot
////            currAngle = orientation.firstAngle;
////            // Delay
////            sleep(20);
////        }
////
////        for (DcMotor motor: motors) motor.setPower(0);
    }

    // function to calculate power for motors given distance and current distance to ensure gradual increase and decrease in motor powers
    // an equation for graph of powers assuming that the highest power is 0.5; graph it in Desmos to see
    static double fWithMaxPow(int x, int n, double maxPow){
        return maxPow * (1 - Math.pow(3.85 * Math.pow(x - n / 2, 2) / (n * n), 1.75)) + 0.1;
    }

    /**
     * Configs:
     *  0: forward
     *  1: right
     *  2: backward
     *  3: left
     * @params degrees linear direction to move the robot (0 degrees is forward
     * @params distance distance at which to move robot
     * @params power speed at which motors will run -- affects speed of movement
     * */
    void move(int config, double distance, double speed){
        setDirection(config);
        for(int i = 0; i < 4 && opModeIsActive(); i++){
            // reset encoders
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            sleep(75); //sleep
            // if we choose to, increment speed gradually to minimize jerking motion
            if(slowDown){
                int currPos = motors[0].getCurrentPosition();
                double pow = f(currPos, totalTick);
                for(DcMotorEx motor : motors) motor.setVelocity(40 * pow * TICKS_PER_INCH);
            }
            else {
                for(DcMotorEx motor : motors) motor.setVelocity(680);
            }
        }
    }

    // function to calculate power for motors given distance and current distance to ensure gradual increase and decrease in motor powers
    // an equation for graph of powers assuming that the highest power is 0.5; graph it in Desmos to see
    static double f(int x, int n){
        return -Math.pow((2.6 * Math.pow(x - n / 2, 2)) / (n * n), 1.75) + 0.6;
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