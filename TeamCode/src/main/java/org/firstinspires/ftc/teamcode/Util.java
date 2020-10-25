package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Util {

    static final double TICKS_PER_INCH = 34.2795262044082261656;
    static final double ARC_LENGTH = 9.487480983 * 5 / 4;
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
    static void rotate(double degrees, double power, DcMotor[] motors, byte config){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //distance that each wheel will travel in rotation type movement
        double arcLength =  ARC_LENGTH * radians;

        setDirection((byte)(config + 4), motors);

        //set motor distances and powers
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition((int)(arcLength * TICKS_PER_INCH));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(power);
        }

        moving(motors, true);

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
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
            motors[i].setTargetPosition((int)(distance * TICKS_PER_INCH));
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
        final int decrements = 20;
        final double ratio = 7.0/12;
        int point = (int)((1 - ratio) * totalTick);
        final double  decrement = motors[0].getPower() / 21;
        final int pointDecrement = point/decrements + 1;
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            int ticksLeft = totalTick - motors[0].getCurrentPosition();
            if (ticksLeft == point && slowDown && motors[0].getPower() > 0.08){
                motors[0].setPower(motors[0].getPower() - decrement);
                motors[1].setPower(motors[0].getPower() - decrement);
                motors[2].setPower(motors[0].getPower() - decrement);
                motors[3].setPower(motors[0].getPower() - decrement);

                telem.addData("Current Power: ", motors[0].getPower());

                point -= pointDecrement;
            }
        }
        return;
    }

    static void moving2(DcMotor[] motors, boolean slowDown, Telemetry telem){
        final int totalTick = motors[0].getTargetPosition();
        final double ratio = 7.0/12;
        int point = (int)((1 - ratio) * totalTick);
        final int pointDecrement = point/20 + 1;
        int currentDec = 0;

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            int ticksLeft = totalTick - motors[0].getCurrentPosition();
            if (ticksLeft == point && slowDown){
                currentDec++;
                for(int i = 0; i < 4; i++){
                    motors[i].setPower(0.5 - Math.pow(currentDec / 30.0, 2));
                }

                telem.addData("Current Power: ", motors[0].getPower());

                point -= pointDecrement;
            }
        }
        return;
    }

    static void moving(DcMotor[] motors, boolean slowDown){

        final int totalTick = motors[0].getTargetPosition();
        final int decrements = 20;
        final double ratio = 7.0/12;
        int point = (int)((1 - ratio) * totalTick);
        final double  decrement = motors[0].getPower() / 21;
        final int pointDecrement = point/decrements + 1;
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            int ticksLeft = totalTick - motors[0].getCurrentPosition();
            if (ticksLeft == point && slowDown && motors[0].getPower() > 0.08){
                motors[0].setPower(motors[0].getPower() - decrement);
                motors[1].setPower(motors[0].getPower() - decrement);
                motors[2].setPower(motors[0].getPower() - decrement);
                motors[3].setPower(motors[0].getPower() - decrement);

                //telem.addData("Current Power: ", motors[0].getPower());

                point -= pointDecrement;
            }
        }
        return;
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
            else if (config == 1 && i <=1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 2 && i % 2 == 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 3 && i >= 2) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 4) motors[i].setDirection(DcMotor.Direction.REVERSE);
        }
    }

}