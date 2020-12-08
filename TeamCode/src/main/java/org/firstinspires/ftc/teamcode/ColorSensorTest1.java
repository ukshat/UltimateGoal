package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Register this Op Mode on the Android phone
@TeleOp(name = "Move backward until white")

public class ColorSensorTest1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

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

        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        fl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorLeft = (RevColorSensorV3) hardwareMap.get("ColorSensorLeft");
        colorRight = (RevColorSensorV3) hardwareMap.get("ColorSensorRight");

        waitForStart();

        fl_motor.setPower(0.3);
        fr_motor.setPower(0.3);
        bl_motor.setPower(0.3);
        br_motor.setPower(0.3);

        while (opModeIsActive()) {
            telemetry.addData("ColorSensorLeft", colorLeft.red() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.red() + "\n");
            telemetry.addData("ColorSensorLeft", colorLeft.blue() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.blue() + "\n");
            telemetry.addData("ColorSensorLeft", colorLeft.green() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.green() + "\n");
            telemetry.addData("ColorSensorLeft", colorLeft.alpha() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.alpha() + "\n");
            telemetry.update();

            if(isWhiteLeft() && isWhiteRight()) {
                fl_motor.setPower(0);
                fr_motor.setPower(0);
                bl_motor.setPower(0);
                br_motor.setPower(0);
                break;
            }
            sleep(20);
        }

    }



    boolean isWhiteLeft (){
        return colorLeft.red() > 700 && colorLeft.blue() > 700 && colorLeft.green() > 700 && colorLeft.alpha() > 700;
    }
    boolean isWhiteRight (){
        return colorRight.red() > 700 && colorRight.blue() > 700 && colorRight.green() > 700 && colorRight.alpha() > 700;
    }


}


