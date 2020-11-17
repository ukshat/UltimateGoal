package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled

public class SimpleLinearTeleOp extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    // created our color sensors
    ColorSensor color1;
    ColorSensor color2;

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

        color1 = hardwareMap.colorSensor.get("ColorSensorLeft");
        color2 = hardwareMap.colorSensor.get("ColorSensorRight");

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

            Gamepad activeGamepad = gamepad1;

            double intake = 1;
            double shooting = gamepad1.right_trigger;
            double wobble = 1;

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

            // if the left stick on gamepad 2 is pressed we change the active gamepad to gamepad2
            if(gamepad2.left_stick_button){
                activeGamepad = gamepad2;
            }

            // if the left stick on gamepad 1 is pressed we change the active gamepad to gamepad1
            if(gamepad1.left_stick_button){
                activeGamepad = gamepad1;
            }

            // if gamepad is equal to a it sets the button for the intake to a
            if(activeGamepad.a){
                leftIntake.setPower(intake);
                rightIntake.setPower(intake);
            }

            // if right trigger is more than 0.4 it sets the button for the shooting
            if(shooting > 0.4){
                rightShoot.setPower(activeGamepad.right_trigger);
                leftShoot.setPower(activeGamepad.right_trigger);
            }

            // if gamepad is equal to y it sets the button for the wobble power to y
            if(activeGamepad.y) {
                wobbleGoal.setPosition(wobble);
            }

            if(activeGamepad.x) {
                // if none of the color sensors see white keep moving forward
                while(color1.red() < 200 && color2.red() < 200){
                    fl_motor.setPower(0.4);
                    fr_motor.setPower(0.4);
                    bl_motor.setPower(0.4);
                    br_motor.setPower(0.4);
                }

                // if the left color sensor see's white but the right doesn't turn the robot left until
                // they both see white
                while(color1.red() > 200 && color2.red() < 200){
                    fl_motor.setPower(0);
                    fr_motor.setPower(0.1);
                    bl_motor.setPower(0.1);
                    br_motor.setPower(0);
                }

                // if the right color sensor see's white but the left doesn't turn the robot left until
                // they both see white
                while(color1.red() < 200 && color2.red() > 200){
                    fl_motor.setPower(0.1);
                    fr_motor.setPower(0);
                    bl_motor.setPower(0);
                    br_motor.setPower(0.1);
                }

                // if both of the color sensors see white move the robot back a little bit so that the
                // robot is on the launch zone and ready to shoot
                while(color1.red() > 200 && color2.red() > 200){
                    fl_motor.setPower(-0.1);
                    fr_motor.setPower(-0.1);
                    bl_motor.setPower(-0.1);
                    br_motor.setPower(-0.1);
                }
            }

            //showing elapsed game time and wheel power
            telemetry.addData("Status", "RunTime: " + runtime.toString());
            telemetry.addData("left front motor", powers[0]);
            telemetry.addData("right front motor", powers[1]);
            telemetry.addData("left back motor", powers[2]);
            telemetry.addData("right back motor", powers[3]);

            telemetry.addData("left intake wheel motor", intake);
            telemetry.addData("right intake wheel motor", intake);
            telemetry.addData("left shooting wheel motor", shooting);
            telemetry.addData("right shooting wheel motor", shooting);
            telemetry.update();

        }
    }

    public double[] calculateMotorPower(double x, double y, double rotation) {
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
        double normalizer = 0.5;
        if(gamepad1.x){
            normalizer = 0.75;

        }

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

}
