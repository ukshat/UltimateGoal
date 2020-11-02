package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled

public class SimpleLinearTeleOp extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    // Create the four motors, one for each mecanum wheel
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private Servo wobbleGoal;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor leftShoot;
    private DcMotor rightShoot;

    @Override
    public void runOpMode() {
        // Find each motor on the hardware map
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        wobbleGoal = hardwareMap.servo.get("Wobble Goal");
        leftIntake = hardwareMap.dcMotor.get("Left Intake Wheel");
        rightIntake = hardwareMap.dcMotor.get("Right Intake Wheel");
        leftShoot = hardwareMap.dcMotor.get("Left Shooting Wheel");
        rightShoot = hardwareMap.dcMotor.get("Right Shooting Wheel");

        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set some motors to reverse so that the motors direction is the same
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftShoot.setDirection(DcMotor.Direction.REVERSE);

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double intakePower = 1;
            double shootingPower = gamepad1.right_trigger;
            double wobblePower = 1;

            // Create variables to hold the direction that the left stick was moved
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            // Create a variable to hold the amount of rotation
            double rotation = gamepad1.right_stick_x;

            // Calculate motor powers and assign to an array
            double[] powers = calculateMotorPower(x, y, rotation);

            // Set powers for each motor
            fl_motor.setPower(powers[0]);
            fr_motor.setPower(powers[1]);
            bl_motor.setPower(powers[2]);
            br_motor.setPower(powers[3]);

            // make sure gamepad control is correct and set powers
            boolean gamepadControl;
            gamepadControl = true;
            changeGamepad(gamepadControl, intakePower, shootingPower, wobblePower);

            //showing elapsed game time and wheel power
            telemetry.addData("Status", "RunTime: " + runtime.toString());
            telemetry.addData("left front motor", powers[0]);
            telemetry.addData("right front motor", powers[1]);
            telemetry.addData("left back motor", powers[2]);
            telemetry.addData("right back motor", powers[3]);

            telemetry.addData("left intake wheel motor", intakePower);
            telemetry.addData("right intake wheel motor", intakePower);
            telemetry.addData("left shooting wheel motor", shootingPower);
            telemetry.addData("right shooting wheel motor", shootingPower);
            telemetry.update();

        }
    }

    public static double[] calculateMotorPower(double x, double y, double rotation) {
        // Assign an amount of power to variable of each motor
        double flPower = x + y + rotation;
        double frPower = -x + y - rotation;
        double blPower = -x + y + rotation;
        double brPower = x + y - rotation;

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
        if (largestPower > 1 || largestPower < -1) {
            normalizer /= Math.abs(largestPower);
        }

        // If the largest power is not out of bounds, there is no need to adjust values

        // Normalize the four powers and return a new array with them
        double[] pows = {(normalizer * powers[0]), (normalizer * powers[1]), (normalizer * powers[2]), (normalizer * powers[3])};
        return pows;
    }

    /**
     * Returns the largest value out of a double array
     *
     * @param powers an array of motor powers
     * @return the largest power out of the array
     */
    public static double findLargest(double[] powers) {
        double largest = Math.abs(powers[0]);
        for (double d : powers) {
            if (Math.abs(d) > largest) {
                largest = Math.abs(d);
            }
        }
        return largest;
    }

    private void changeGamepad(boolean gamepad, double intake, double shooting, double wobble){
        // sets a variable to false if left stick button is pressed on gamepad 2
        if(gamepad2.left_stick_button){
            gamepad = false;
        }
        // sets a variable to false if left stick button is pressed on gamepad 1
        if(gamepad1.left_stick_button){
            gamepad = true;
        }
        // if gamepad is equal to true it sets these powers and buttons
        if(gamepad){
            // if gamepad is equal to true and a it sets the button for the intake to the a on the gamepad
            if(gamepad1.a){
                leftIntake.setPower(intake);
                rightIntake.setPower(intake);
            }

            // if gamepad is equal to true and right trigger is more than 0.4 it sets the button
            // for the shooting to the right trigger on the gamepad
            if(shooting > 0.4){
                rightShoot.setPower(gamepad1.right_trigger);
                leftShoot.setPower(gamepad1.right_trigger);
            }

            // if gamepad is equal to true and y it sets the button for the shooting to the y
            // on the gamepad
            if(gamepad1.y) {
                wobbleGoal.setPosition(wobble);
            }
        }
        // if gamepad is not set to true it is set to false which is why in any other scenario
        // we set all the buttons and power to gamepad 2 rather than 1
        else {
            if(gamepad2.a){
                leftIntake.setPower(intake);
                rightIntake.setPower(intake);
            }

            if(shooting > 0.4){
                rightShoot.setPower(gamepad1.right_trigger);
                leftShoot.setPower(gamepad1.right_trigger);
            }

            if(gamepad2.y) {
                wobbleGoal.setPosition(wobble);
            }
        }
    }
}
