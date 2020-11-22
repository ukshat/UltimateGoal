package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Register this Op Mode on the Android phone
@TeleOp(name = "Color Sensor Test2")

public class ColorSensorTest2 extends LinearOpMode {

    RevColorSensorV3 colorLeft;
    RevColorSensorV3 colorRight;

    DcMotor fl_motor;
    DcMotor fr_motor;
    DcMotor bl_motor;
    DcMotor br_motor;

    @Override
    public void runOpMode() throws InterruptedException {

        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        colorLeft = (RevColorSensorV3) hardwareMap.get("ColorSensorLeft");
        colorRight = (RevColorSensorV3) hardwareMap.get("ColorSensorRight");

        while(opModeIsActive()) {
            // if none of the color sensors see white keep moving forward
            while(colorLeft.red() < 200 && colorRight.red() < 200){
                fl_motor.setPower(-0.4);
                fr_motor.setPower(-0.4);
                bl_motor.setPower(-0.4);
                br_motor.setPower(-0.4);
            }

            // if the left color sensor see's white but the right doesn't turn the robot left until
            // they both see white
            while(colorLeft.red() < 200 && colorRight.red() > 200){
                fl_motor.setPower(0);
                fr_motor.setPower(-0.1);
                bl_motor.setPower(0);
                br_motor.setPower(-0.1);
            }

            // if the right color sensor see's white but the left doesn't turn the robot left until
            // they both see white
            while(colorLeft.red() > 200 && colorRight.red() < 200){
                fl_motor.setPower(-0.1);
                fr_motor.setPower(0);
                bl_motor.setPower(-0.1);
                br_motor.setPower(0);
            }

            // if both of the color sensors see white move the robot back a little bit so that the
            // robot is on the launch zone and ready to shoot
            while(colorLeft.red() > 200 && colorRight.red() > 200){
                fl_motor.setPower(0.1);
                fr_motor.setPower(0.1);
                bl_motor.setPower(0.1);
                br_motor.setPower(0.1);
            }
        }
    }
}
