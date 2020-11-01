package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled


public class SimpleIterativeTeleOp extends OpMode {
    //declaring opmode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private Servo wobbleGoal;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor leftShoot;
    private DcMotor rightShoot;

    //press init
    @Override
    public void init() {
        //initialize telemetry
        telemetry.addData("Status", "Initialized");

        //initialize leftdrive and rightdrive
        //find hardware map of motors
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        wobbleGoal = hardwareMap.servo.get("Wobble Goal");
        leftIntake = hardwareMap.dcMotor.get("Left Intake");
        rightIntake = hardwareMap.dcMotor.get("Right Intake");
        leftShoot = hardwareMap.dcMotor.get("Left Shoot");
        rightShoot = hardwareMap.dcMotor.get("Right Shoot");


        //reverse some motors
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftShoot.setDirection(DcMotor.Direction.REVERSE);
    }

    //reset elapsed time
    //code starts running after driver hits play
    public void start() {
        runtime.reset();
    }

    //code runs after driver hits play but before driver hits stop
    @Override
    public void loop() {
        // Create variables to hold the direction that the left stick was moved
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        double shootingPower = gamepad1.right_trigger;
        double intakePower = 1;
        double wobblePower = 1;

        // Create a variable to hold the amount of rotation
        double rotation = gamepad1.right_stick_x;

        // Calculate motor powers and assign to an array
        double[] powers = calculateMotorPower(x, y, rotation);

        boolean gamepadControl;
        gamepadControl = true;
        changeGamepad(gamepadControl, intakePower, shootingPower, wobblePower);

        // Set powers for each motor
        fl_motor.setPower(powers[0]);
        fr_motor.setPower(powers[1]);
        bl_motor.setPower(powers[2]);
        br_motor.setPower(powers[3]);

        //showing elapsed game time and wheel power
        telemetry.addData("Status", "RunTime: " + runtime.toString());
        telemetry.addData("left front motor", powers[0]);
        telemetry.addData("right front motor", powers[1]);
        telemetry.addData("left back motor", powers[2]);
        telemetry.addData("right back motor", powers[3]);
        telemetry.addData("left shooting wheel power", shootingPower);
        telemetry.addData("right shooting wheel power", shootingPower);
        telemetry.addData("left intake wheel power", intakePower);
        telemetry.addData("right intake wheel power", intakePower);
        telemetry.update();
    }

    //happens once driver hits stop
    public void stop() {

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

    private void changeGamepad(boolean gamepad, double intake, double shooting, double wobble) {
        if (gamepad2.left_stick_button) {
            gamepad = false;
        }
        if (gamepad1.left_stick_button) {
            gamepad = true;
        }
        if (gamepad) {
            if (gamepad1.a) {
                leftIntake.setPower(intake);
                rightIntake.setPower(intake);
            }

            if (shooting > 0.4) {
                rightShoot.setPower(gamepad1.right_trigger);
                leftShoot.setPower(gamepad1.right_trigger);
            }

            if (gamepad1.y) {
                wobbleGoal.setPosition(wobble);
            }
        } else {
            if (gamepad2.a) {
                leftIntake.setPower(intake);
                rightIntake.setPower(intake);
            }

            if (shooting > 0.4) {
                rightShoot.setPower(gamepad1.right_trigger);
                leftShoot.setPower(gamepad1.right_trigger);
            }

            if (gamepad2.y) {
                wobbleGoal.setPosition(wobble);
            }
        }
    }
}