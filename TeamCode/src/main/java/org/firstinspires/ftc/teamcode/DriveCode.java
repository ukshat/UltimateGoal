package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Register this Op Mode on the Android phone
@TeleOp (name = "Drive Code", group = "Op Modes")
public class DriveCode extends LinearOpMode {

    // Create the four motors, one for each mecanum wheel
    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    @Override
    public void runOpMode() {
        // Find each motor on the hardware map
        tl_motor = hardwareMap.dcMotor.get("LeftFront");
        tr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        waitForStart();

        while(opModeIsActive()){

            // Create variables to hold the direction that the left stick was moved
            double xPos = gamepad1.left_stick_x;
            double yPos = gamepad1.left_stick_y;

            // Create a variable to hold the amount of rotation
            double rot = gamepad1.right_stick_x;

            double[] powers = calculateMotorPower(xPos, yPos, rot);
            // Apply the scalar to each of the powers and set the motors to that power
            tl_motor.setPower(powers[0]);
            tr_motor.setPower(powers[1]);
            bl_motor.setPower(powers[2]);
            br_motor.setPower(powers[3]);

        }
    }

    public static double[] calculateMotorPower(double x, double y, double rotation) {
        // Assign an amount of power to variable of each motor
        double tlPower = x + y + rotation;
        double trPower = -x + y - rotation;
        double blPower = -x + y + rotation;
        double brPower = x + y - rotation;

        // In the case that any power is out of the bounds of setPower(), we have to scale
        // all powers down so that the largest power is exactly 1, and the rest are
        // proportional to the largest power

        // Find the largest power of the four powers
        double[] powers = {tlPower, trPower, blPower, brPower};
        double largestPower = findLargest(powers);

        // Create a variable to scale each power down
        double normalizer;

        // If the largest power happens to be out of bounds, assign to scalar a value such
        // that the largest power will be scaled down to exactly one
        if(largestPower > 1 || largestPower < -1){
            normalizer = 1 / Math.abs(largestPower);
        }

        // If the largest power is not out of bounds, there is no need to adjust values
        else {
            normalizer = 1;
        }

        return new double[]{(normalizer * tlPower), (normalizer * trPower), (normalizer * blPower), (normalizer * brPower)};
    }

    /**
     * Returns the largest value out of a double array
     * @param powers   an array of motor powers
     * @return         the largest power out of the array
     */
    public static double findLargest(double[] powers){
        double largest = powers[0];
        for(double d: powers){
            if(d > largest){
                largest = d;
            }
        }
        return largest;
    }

}
