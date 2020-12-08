package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Register this Op Mode on the Android phone
@TeleOp(name = "Move backward until white")

public class ColorSensorTest3 extends LinearOpMode {

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

        fl_motor.setPower(0.5);
        fr_motor.setPower(0.5);
        bl_motor.setPower(0.5);
        br_motor.setPower(0.5);

        while (!isWhiteLeft()) {
            sleep(20);
        }

        fl_motor.setPower(-0.5);
        fr_motor.setPower(-0.5);
        bl_motor.setPower(-0.5);
        br_motor.setPower(-0.5);

        sleep(500);

    }

    boolean isWhiteLeft (){
        return colorLeft.red() > 700 && colorLeft.blue() > 700 && colorLeft.green() > 700 && colorLeft.alpha() > 700;
    }

}