package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ColorSensorTest1 extends LinearOpMode {

    ColorSensor color1;
    ColorSensor color2;

    DcMotor fl_motor;
    DcMotor fr_motor;
    DcMotor bl_motor;
    DcMotor br_motor;

    //@Override
    public void runOpMode() throws InterruptedException {
        while(color1.red() < 200 && color2.red() < 200){
            fl_motor.setPower(0);
            fr_motor.setPower(0.3);
            bl_motor.setPower(0);
            br_motor.setPower(0.3);
        }
    }
}
