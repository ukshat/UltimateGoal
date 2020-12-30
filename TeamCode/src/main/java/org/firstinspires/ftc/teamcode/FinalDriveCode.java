package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Drive Code")
public class FinalDriveCode extends LinearOpMode {

    // Create the four motors, one for each mecanum wheel
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors based on the hardware map
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        // Run with encoders to ensure that errors don't happen
        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the motors on the right in reverse because the motors are flipped
        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            // Create variables to hold the direction that the left stick was moved
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            // Create a variable to hold the amount of rotation
            double rotation = gamepad1.right_stick_x;

            // Calculate motor powers and assign to an array
            double[] powers = Util.calculateMotorPower(x, y, rotation);

            // Set powers for each motor
            fl_motor.setPower(powers[0]);
            fr_motor.setPower(powers[1]);
            bl_motor.setPower(powers[2]);
            br_motor.setPower(powers[3]);

            // Show current motor powers on the driver station
            telemetry.addData("left front motor", powers[0] + "\n");
            telemetry.addData("right front motor", powers[1] + "\n");
            telemetry.addData("left back motor", powers[2] + "\n");
            telemetry.addData("right back motor", powers[3] + "\n");
            telemetry.update();
        }

    }

    double[] calculateMotorPower(double x, double y, double rotation) {

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

    double findLargest(double[] powers){
        double largest = Math.abs(powers[0]);
        for(double d: powers) if(Math.abs(d) > largest) largest = Math.abs(d);
        return largest;
    }

}
