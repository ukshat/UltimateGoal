package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Register this Op Mode on the Android phone
@TeleOp(name = "Find values for the color sensors")

public class ColorSensorTest0 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    RevColorSensorV3 colorLeft;
    RevColorSensorV3 colorRight;

    DcMotor fl_motor;
    DcMotor fr_motor;
    DcMotor bl_motor;
    DcMotor br_motor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorLeft = (RevColorSensorV3) hardwareMap.get("ColorSensorLeft");
        colorRight = (RevColorSensorV3) hardwareMap.get("ColorSensorRight");

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("ColorSensorLeft", colorLeft.red() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.red() +  "\n");
            telemetry.addData("ColorSensorLeft", colorLeft.blue() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.blue() +  "\n");
            telemetry.addData("ColorSensorLeft", colorLeft.green() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.green() +  "\n");
            telemetry.addData("ColorSensorLeft", colorLeft.alpha() + "\n");
            telemetry.addData("ColorSensorRight", colorRight.alpha() +  "\n");
            telemetry.update();

            sleep(500);

        }
        sleep(500);
    }

    boolean isWhiteLeft (){
        return colorLeft.red() > 700 && colorLeft.blue() > 700 && colorLeft.green() > 700 && colorLeft.alpha() > 700;
    }
    boolean isWhiteRight (){
        return colorRight.red() > 700 && colorRight.blue() > 700 && colorRight.green() > 700 && colorRight.alpha() > 700;
    }

}



