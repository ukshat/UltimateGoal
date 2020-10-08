package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="all motors 1 sec each")
public class Test1 extends LinearOpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.dcMotor.get("LeftFront");
        fr = hardwareMap.dcMotor.get("RightFront");
        bl = hardwareMap.dcMotor.get("LeftRear");
        br = hardwareMap.dcMotor.get("RightRear");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        fl.setPower(0.5);
        Thread.sleep(1000);
        fl.setPower(0);
        fr.setPower(0.5);
        Thread.sleep(1000);
        fr.setPower(0);
        bl.setPower(0.5);
        Thread.sleep(1000);
        bl.setPower(0);
        br.setPower(0.5);
        Thread.sleep(1000);
        br.setPower(0);
    }
}
