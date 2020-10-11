package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="move right 2 sec")
public class Test2 extends LinearOpMode {

    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        setPowers(0.5);
        Thread.sleep(2000);
        setPowers(0);
    }
    void setPowers(double power){
        fl_motor.setPower(power);
        fr_motor.setPower(-power);
        bl_motor.setPower(-power);
        br_motor.setPower(power);
    }

}