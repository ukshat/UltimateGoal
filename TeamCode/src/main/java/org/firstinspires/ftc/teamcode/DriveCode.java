package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveCode extends LinearOpMode {

    private DcMotor tl_motor;
    private DcMotor tr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    @Override
    public void runOpMode() {
        tl_motor = hardwareMap.dcMotor.get("tl_motor");
        tr_motor = hardwareMap.dcMotor.get("tr_motor");
        bl_motor = hardwareMap.dcMotor.get("bl_motor");
        br_motor = hardwareMap.dcMotor.get("br_motor");

        tr_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        br_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            double xPos = gamepad1.left_stick_x;
            double yPos = gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            double tlPower = xPos + yPos + rot;
            double trPower = xPos - yPos - rot;
            double blPower = xPos - yPos + rot;
            double brPower = xPos + yPos - rot;

            double[] powers = {tlPower, trPower, blPower, brPower};
            double largestPower = findLargest(powers);

            double scalar;
            if(largestPower > 1 || largestPower < -1){
                scalar = 1 / Math.abs(largestPower);
            }
            else {
                scalar = 1;
            }

            tl_motor.setPower(scalar * tlPower);
            tr_motor.setPower(scalar * trPower);
            bl_motor.setPower(scalar * blPower);
            br_motor.setPower(scalar * brPower);

        }
    }
    public double findLargest(double[] powers){
        double largest = powers[0];
        for(double d: powers){
            if(d > largest){
                largest = d;
            }
        }
        return largest;
    }

}
