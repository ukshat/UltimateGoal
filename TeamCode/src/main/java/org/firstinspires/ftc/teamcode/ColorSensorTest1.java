package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Register this Op Mode on the Android phone
@TeleOp(name = "Color Sensor Test1")

public class ColorSensorTest1 extends LinearOpMode {

    private static RevColorSensorV3 colorLeft;
    private static RevColorSensorV3 colorRight;

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

        while(opModeIsActive()){
            if(gamepad1.x){
                while(isWhiteLeft() && isWhiteRight()){
                    fl_motor.setPower(0);
                    fr_motor.setPower(-0.3);
                    bl_motor.setPower(0);
                    br_motor.setPower(-0.3);
                }
            }
        }
    }
    public static boolean isWhiteLeft (){
        boolean isWhite = false;
        if (colorLeft.red() < 700 && colorLeft.blue() < 700 && colorLeft.green() < 700 && colorLeft.alpha() < 700) {
            isWhite = true;
        }
        return isWhite;
    }
    public static boolean isWhiteRight (){
        boolean isWhite = false;
        if (colorRight.red() < 700 && colorRight.blue() < 700 && colorRight.green() < 700 && colorRight.alpha() < 700) {
            isWhite = true;
        }
        return isWhite;
    }
}

