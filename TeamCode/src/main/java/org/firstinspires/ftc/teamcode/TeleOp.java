package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled

public class TeleOp extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    // Create the four motors, one for each mecanum wheel
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    private DcMotor intake;
    private DcMotor shooter;
    private Servo intakeAssist;
    private WobbleMech wobbleMech;

    private AnalogInput inp;
    private DcMotorEx arm;

    @Override
    public void runOpMode() {
        // Find each motor on the hardware map
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        intakeAssist = hardwareMap.servo.get("shoot");

        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        inp = hardwareMap.analogInput.get("wobblepot");
        arm = (DcMotorEx) hardwareMap.get("wobblemotor");

        //set some motors to reverse so that the motors direction is the same
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        //wait for driver to press play
        wobbleMech = new WobbleMech();
        waitForStart();
        runtime.reset();

        double targetPosition = inp.getVoltage();

        while (opModeIsActive()) {

            // Create a variable to hold the amount of rotation
            double rotation = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            // Calculate motor powers and assign to an array
            double[] powers = calculateMotorPower(x, y, rotation);

            // Set powers for each motor
            fl_motor.setPower(powers[0]);
            fr_motor.setPower(powers[1]);
            bl_motor.setPower(powers[2]);
            br_motor.setPower(powers[3]);

            double intakePower = 1;
            double intakeAssistPower = 1;
            double shootingPower = gamepad2.right_trigger;
            waitForStart();

            // if gamepad is equal to a it sets the button for the intake to a
            if (gamepad1.a) {
                intake.setPower(intakePower);
            } else {
                intake.setPower(0);
            }

            // if gamepad is equal to a it sets the button for the intake to right trigger
            if (shootingPower > 0.5) {
                shooter.setPower(1.0);
                intakeAssist.setPosition(intakeAssistPower);
            } else {
                shooter.setPower(0);
                intakeAssist.setPosition(0);
            }


            if(gamepad2.b)
                targetPosition = wobbleMech.upperBound;

            if(gamepad2.x)
                targetPosition = wobbleMech.lowerBound;

            if(gamepad2.a)
                wobbleMech.open();
            else
                wobbleMech.close();

            byte temp = 0;
            if(targetPosition == wobbleMech.upperBound)
                temp = -1;
            else if(targetPosition == wobbleMech.lowerBound)
                temp = 1;


            if(Math.abs(targetPosition - inp.getVoltage()) < 0.15)
                arm.setVelocity(0);
            else if(temp == -1 && inp.getVoltage() < wobbleMech.upperBound)
                arm.setVelocity(-100);
            else if(temp == 1 && inp.getVoltage() > wobbleMech.lowerBound)
                arm.setVelocity(100);



            //showing elapsed game time and wheel power
            telemetry.addData("Status", "RunTime: " + runtime.toString());
            telemetry.addData("left front motor", powers[0]);
            telemetry.addData("right front motor", powers[1]);
            telemetry.addData("left back motor", powers[2]);
            telemetry.addData("right back motor", powers[3]);

            telemetry.addData("intake wheel motor", intakePower + "\n");
            telemetry.addData("shooting wheel motor", shootingPower + "\n");
            telemetry.addData("intake assist servo", intakePower + "\n");
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
        return new double[]{(normalizer * powers[0]), (normalizer * powers[1]), (normalizer * powers[2]), (normalizer * powers[3])};
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

    class WobbleMech {
        private final Servo clasp;
        private boolean isClosed;
        final double safeBound = 0.62, lowerBound = 1.0, upperBound = 2.21;

        public WobbleMech() {
            clasp = hardwareMap.servo.get("wobbleservo");
            clasp.setDirection(Servo.Direction.FORWARD);
        }

        public void close (){
            clasp.setPosition(0);
            isClosed = true;
        }

        public void open (){
            clasp.setPosition(1);
            isClosed = false;
        }
    }

}
