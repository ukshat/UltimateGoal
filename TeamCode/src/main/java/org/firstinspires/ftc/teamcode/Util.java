package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Util {

    static final double TICKS_PER_INCH = 34.2795262044082261656;
    static final double ARC_LENGTH = 9.487480983 * (5.0 / 4) * (11.0/10) * (5/5.25);
    static final double TICK_LENGTH = 0.029171931783333794357;
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;

    /**
     * Calculates the number of ticks per inch the robot moves
     * @return a double value containing ticks per inch
     */
    public static double getTicksPerInch(){
        int encoderTicks = 28;
        int gearboxRatio = 40;
        double sprocketRatio = 2.6;
        int wheelRadius = 2;

        double circumference = wheelRadius * 2 * Math.PI;
        double ticksPerRev = encoderTicks * gearboxRatio / sprocketRatio;

        double ticksPerInch = ticksPerRev / circumference;
        return ticksPerInch;

    }

    /**
     * Calculates four motor powers for each wheel of a mecanum robot based on movement of
     * the left and right joysticks on a gamepad
     * @param x         A value from -1 to 1 that represents the left stick's left/right movement
     * @param y         A value from -1 to 1 that represents the left stick's up/down movement
     * @param rotation  A value from -1 to 1 that represents the right stick's left/right movement
     * @return          An array of length 4 with motor powers from -1 to 1, ordered fl-fr-bl-br
     */
    public static double[] calculateMotorPower(double x, double y, double rotation) {

        // Find the distance that the stick is moved
        double r = Math.hypot(x, y);

        // Find the angle in which the robot is moving and subtract 45 degrees
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;

        // Calculate motor powers based on angle of the robot

        double[] powers = new double[4];

        powers[0] = r * Math.cos(robotAngle) + rotation;
        powers[1] = r * Math.sin(robotAngle) - rotation;
        powers[2] = r * Math.sin(robotAngle) + rotation;
        powers[3] = r * Math.cos(robotAngle) - rotation;

        // In the case that any power is out of the bounds of setPower(), we have to scale
        // all powers down so that the largest power is exactly 1, and the rest are
        // proportional to the largest power

        // Find the largest power of the four powers
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
        double[] pows = {(normalizer * powers[0]), (normalizer * powers[1]), (normalizer * powers[2]), (normalizer * powers[3])};
        return pows;

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

    /**
     * rotates the robot a certain amount of degrees at a given speed
     *
     * @params degrees amount in degrees to rotate the robot (must be between -180 and 180)
     * @params power speed at which motors will run -- affects speed of rotation
     * */
    static void rotate(double degrees, BNO055IMU imu, DcMotor[] motors){

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        final double startAngle = angle;
        if (degrees < 0){
            setDirection((byte)(4), motors );
            while (angle > degrees*0.925){
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = orientation.firstAngle;
                for (DcMotor motor: motors){
                    motor.setPower(0.2);
                }
            }

        }

        else{

            setDirection((byte)(5), motors );
            while (angle < degrees*0.925){
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = orientation.firstAngle;
                for (DcMotor motor: motors){
                    motor.setPower(0.2);
                }
            }

        }
        for (DcMotor motor: motors){
            motor.setPower(0);
        }


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
    static void move(byte config, double distance, double speed, DcMotor[] motors){
        setDirection(config, motors);
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setPower(speed);
            motors[i].setTargetPosition((int)(distance * TICKS_PER_INCH * (config % 2 == 1 ? HORIZONTAL_STRAFE : 1)));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving(motors, true);

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    static void moving(DcMotor[] motors, boolean slowDown, Telemetry telem){
        final int totalTick = motors[0].getTargetPosition();

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            try {Thread.sleep(75);} catch (InterruptedException e) {} //sleep
            if(slowDown){
                int currPos = motors[0].getCurrentPosition();

                double pow = f(currPos, totalTick);

                for(DcMotor motor : motors) motor.setPower(pow);

                telem.addData("Current power: ", pow + "\n");
                telem.update();
             }
        }
    }

    static void moving(DcMotor[] motors, boolean slowDown){
        final int totalTick = motors[0].getTargetPosition();

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            try {Thread.sleep(75);} catch (InterruptedException e) {} //sleep
            if(slowDown){
                int currPos = motors[0].getCurrentPosition();

                double pow = f(currPos, totalTick);

                for(DcMotor motor : motors) motor.setPower(pow);
            }
        }
    }

    static double f(int x, int n){
        return -Math.pow((2.8 * Math.pow(x - n / 2, 2)) / (n * n), 2) + 0.5;
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
     * @param motors
     */
    static void setDirection(byte config, DcMotor[] motors){

        for(int i = 0; i < 4; i++){
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            if (config == 0 && i % 2 == 0) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 1 && i >= 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 2 && i % 2 == 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 3 && i <= 3) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 4) motors[i].setDirection(DcMotor.Direction.REVERSE);
        }
    }

}