package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Util {

    static final double TICKS_PER_INCH = 34.2795262044082261656;
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;
    static final double TILE_LENGTH = 23.5;

    /**
     * Calculates the number of ticks per inch the robot moves
     * @deprecated
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
        for(double d: powers) if(Math.abs(d) > largest) largest = Math.abs(d);
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
        if (degrees < 0) {
            setDirection(4, motors );
            while (angle > degrees * 0.925) {
                try { Thread.sleep(20); } catch (InterruptedException e) {}
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = orientation.firstAngle;
                for (DcMotor motor: motors) motor.setPower(0.2);
            }
        }
        else {
            setDirection(5, motors);
            while (angle < degrees * 0.925){
                try { Thread.sleep(20); } catch (InterruptedException e) {}
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = orientation.firstAngle;
                for (DcMotor motor: motors) motor.setPower(0.2);
            }
        }
        for (DcMotor motor: motors) motor.setPower(0);
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
    static void move(int config, double distance, double speed, DcMotor[] motors){
        setDirection(config, motors);
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setPower(speed);
            motors[i].setTargetPosition((int)(distance * TICKS_PER_INCH * (config % 2 == 1 ? HORIZONTAL_STRAFE : 1)));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving(motors, true);

        //stop motors
        for(DcMotor motor: motors) motor.setPower(0);

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
    static void setDirection(int config, DcMotor[] motors){

        for(int i = 0; i < 4; i++){
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            if (config == 0 && i % 2 == 0) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 1 && i >= 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 2 && i % 2 == 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 3 && i <= 3) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 4) motors[i].setDirection(DcMotor.Direction.REVERSE);
        }
    }

    double map(double from){
        return from / HORIZONTAL_STRAFE;
    }

    void move(double degrees, double distance, double speed, DcMotorEx[] motors){
        setDirection(0, motors);
        //inch to ticks
        distance *= TICKS_PER_INCH;

        //deg to rad
        degrees = Math.toRadians(degrees);

        //x and y of mapped point on ellipse
        double x = Math.cos(degrees) * distance, y = map(Math.sin(degrees)) * distance;

        //rotate axis about origin by 45 deg ccw
        degrees = Math.asin(y/distance) + Math.PI/4;

        //rewrite x and y to be mapped to new axis
        x = Math.cos(degrees) * distance;
        y = map(Math.sin(degrees)) * distance;

        /*Front Left, Front Right, Back Left, Back Right*/
        for(int i = 0; i < 4; i++) motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //FL
        motors[0].setTargetPosition((int) y);
        //FR
        motors[1].setTargetPosition((int) x);
        //BL
        motors[2].setTargetPosition((int) x);
        //BR
        motors[3].setTargetPosition((int) y);

        for(int i = 0; i < 4; i++) motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set x and y to be between -0.5 and 0.5
        double max = Math.max(Math.abs(x), Math.abs(y));
        x = x / max * speed;
        y = y / max * speed;

        while ((motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy())){
            // delay
            try{
                Thread.sleep(75);
            } catch (InterruptedException ignored){}

            // current position of motor travel
            int currPosX = motors[1].getCurrentPosition();
            int currPosY = motors[0].getCurrentPosition();

            // calculate ticks per second
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

        for(DcMotorEx m : motors) m.setVelocity(0);
    }

    // function to calculate power for motors given distance and current distance to ensure gradual increase and decrease in motor powers
    // an equation for graph of powers assuming that the highest power is 0.5; graph it in Desmos to see
    static double fWithMaxPow(int x, int n, double maxPow){
        return maxPow * (1 - Math.pow(3.85 * Math.pow(x - n / 2, 2) / (n * n), 1.75));
    }


}