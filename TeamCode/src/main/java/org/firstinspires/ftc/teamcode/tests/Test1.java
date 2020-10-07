package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="fl motor 1 sec")
public class Test1 extends LinearOpMode {
    DcMotor fl;
    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.dcMotor.get("LeftFront");
        waitForStart();
        fl.setPower(0.5);
        Thread.sleep(1000);
        fl.setPower(0);
    }
}
