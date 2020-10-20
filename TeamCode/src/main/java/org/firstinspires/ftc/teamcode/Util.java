package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Util {

    static final double TICKS_PER_INCH = 34.2795262044082261656;
    static final double ARC_LENGTH = 9.487480983;
    static final double TICK_LENGTH = 0.029171931783333794357;

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

    /**
     * rotates the robot a certain amount of degrees at a given speed
     *
     * @params degrees amount in degrees to rotate the robot (must be between -180 and 180)
     * @params power speed at which motors will run -- affects speed of rotation
     * */
    static void rotate(double degrees, double power, DcMotor[] motors, byte[] motorDirs){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //motor powers for rotation will always be like this
        byte[] pows = {1, -1, 1, -1};

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
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition((int)(arcLength * TICKS_PER_INCH * motorDirs[i] * pows[i]));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(power);
        }

        moving(motors);

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    /**
     * moves the robot in a given linear direction for a given distance and speed
     *
     * @params degrees linear direction to move the robot (0 degrees is forward
     * @params distance distance at which to move robot
     * @params power speed at which motors will run -- affects speed of movement
     * */
    static void move(double degrees, double distance, double power, DcMotor[] motors, byte[] motorDirs){
        //convert degrees to radians
        double radians = -degrees * Math.PI / 180;

        //calculate dimensions of tentative unit right triangle described by given parameters
        double x = power * Math.cos(radians);
        double y = power * Math.sin(radians);

        //calculate motor powers
        double[] pows = calculateMotorPower(x, y, 0);

        //assign powers and distances to motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(Math.abs(pows[i]));
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition((int)(distance * TICKS_PER_INCH * motorDirs[i] * pows[i] / Math.abs(pows[i])));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving(motors);

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    static void moving(DcMotor[] motors , boolean slowDown){

        final int firstTick = Math.abs (motors[0].getTargetPosition());
        final double [] powers = [motors[0].getPower(), motors[1].getPower(), motors[2].getPower(), motors[3].getPower()];
        final int decrements = 20;
        final double ratio = 7.0/12;
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){

            if (slowDown){
                //get the total distance and subtract the distance we travelled. Multiply the result
                // by the total distance



            }

        }
        return;
    }

    static byte[] setDefaultDirs(byte[] motorDirs){
        byte[] arr = {-1, 1, -1, 1};
        return arr;
    }
}