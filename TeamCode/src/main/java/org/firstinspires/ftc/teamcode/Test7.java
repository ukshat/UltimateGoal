package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Test7 extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors;

    static final double TILE_LENGTH = 23.5;
    static final double TICK_LENGTH = 0.029171931783333795;

    @Override
    public void runOpMode() throws InterruptedException {
        //init motors
        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        //set modes & zero power behaviour
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //set left side motors to rotate in opposite direction
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        rotate(360 * 5, 0.5);
    }

    /**
     * rotates the robot a certain amount of degrees at a given speed
     *
     * @params degrees amount in degrees to rotate the robot (must be between -180 and 180)
     * @params power speed at which motors will run -- affects speed of rotation
     * */
    void rotate(double degrees, double power){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //motor powers for rotation will always be like this
        double[] pows = {power, -power, power, -power};

        //if turning counter clockwise, reverse motor powers
        if (degrees < 0){
            for(int i = 0; i < 4; i++){
                pows[i] *= -1;
            }
        }

        //distance that each wheel will travel in rotation type movement
        double arcLength =  9.487480983 * radians;

        //set motor distances and powers
        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(arcLength * TICK_LENGTH));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(pows[i]);
        }

        moving();

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    /**
     * moves the robot in a given linear direction for a given distance and speed
     *
     * @params degrees linear direction to move the robot
     * @params distance distance at which to move robot
     * @params power speed at which motors will run -- affects speed of movement
     * */
    void move(double degrees, double distance, double power){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //calculate dimensions of tentative unit right triangle described by given parameters
        double x = power * Math.cos(radians);
        double y = power * Math.sin(radians);

        //calculate motor powers
        double[] pows = calculateMotorPower(x, y, 0);

        //assign powers and distances to motors
        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(distance * TICK_LENGTH));
            motors[i].setPower(pows[i]);
        }

        moving();

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    void moving(){
        //runs infinite loop until motor stops moving
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){}
        return;
    }

    public static double[] calculateMotorPower(double x, double y, double rotation) {

        // Find the distance that the stick is moved
        double r = Math.hypot(x, y);

        // Find the angle in which the robot is moving and subtract 45 degrees
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;

        // Calculate motor powers based on angle of the robot
        double flPower = r * Math.cos(robotAngle) + rotation;
        double frPower = r * Math.sin(robotAngle) - rotation;
        double blPower = r * Math.sin(robotAngle) + rotation;
        double brPower = r * Math.cos(robotAngle) - rotation;

        // In the case that any power is out of the bounds of setPower(), we have to scale
        // all powers down so that the largest power is exactly 1, and the rest are
        // proportional to the largest power

        // Find the largest power of the four powers
        double[] powers = {flPower, frPower, blPower, brPower};
        double largestPower = findLargest(powers);

        // Create a variable to scale each power down
        double normalizer = 1;

        // If the largest power happens to be out of bounds, assign to scalar a value such
        // that the largest power will be scaled down to exactly one
        if(largestPower > 1 || largestPower < -1){
            normalizer /= Math.abs(largestPower);
        }

        // If the largest power is not out of bounds, there is no need to adjust values

        // Normalize the four powers and return a new array with them
        return new double[]{(normalizer * flPower), (normalizer * frPower), (normalizer * blPower), (normalizer * brPower)};

    }
    /**
     * Returns the largest value out of a double array
     * @param powers   an array of motor powers
     * @return         the largest power out of the array
     */
    public static double findLargest(double[] powers){
        double largest = Math.abs(powers[0]);
        for(double d: powers){
            if(Math.abs(d) > largest){
                largest = Math.abs(d);
            }
        }
        return largest;
    }
}
