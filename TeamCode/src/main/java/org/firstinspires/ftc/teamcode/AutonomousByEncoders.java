package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonomousByEncoders extends LinearOpMode {

    DcMotor fl_motor;
    DcMotor fr_motor;
    DcMotor bl_motor;
    DcMotor br_motor;

    static final double TICK_LENGTH = 100 / Math.PI;

    @Override
    public void runOpMode() throws InterruptedException {
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);

        fl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        double distance = 304.8;

        fl_motor.setTargetPosition((int)(distance * TICK_LENGTH));
        fr_motor.setTargetPosition((int)(distance * TICK_LENGTH));
        bl_motor.setTargetPosition((int)(distance * TICK_LENGTH));
        br_motor.setTargetPosition((int)(distance * TICK_LENGTH));

        fl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl_motor.setPower(-1);
        fr_motor.setPower(1);
        bl_motor.setPower(1);
        br_motor.setPower(-1);

        moving();

        fl_motor.setPower(0);
        fr_motor.setPower(0);
        bl_motor.setPower(0);
        br_motor.setPower(0);

    }

    void moving(){
        while (fl_motor.isBusy() || fr_motor.isBusy() || bl_motor.isBusy() || br_motor.isBusy()){

        }
        return;
    }

}
