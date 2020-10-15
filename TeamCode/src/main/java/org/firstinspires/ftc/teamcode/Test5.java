package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Forward 1 yard")
public class Test5 extends LinearOpMode {

    //1 inch is 100/pi ticks
    static final double TICK_LENGTH = 34.2795262044082261656;
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors based on the hardware map
        fl_motor = hardwareMap.dcMotor.get("LeftFront");
        fr_motor = hardwareMap.dcMotor.get("RightFront");
        bl_motor = hardwareMap.dcMotor.get("LeftRear");
        br_motor = hardwareMap.dcMotor.get("RightRear");

        // Run with encoders to ensure that errors don't happen
        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the motors on the right in reverse because the motors are flipped
        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        fl_motor.setTargetPosition((int)(36 * TICK_LENGTH));
        fr_motor.setTargetPosition((int)(36 * TICK_LENGTH));
        bl_motor.setTargetPosition((int)(36 * TICK_LENGTH));
        br_motor.setTargetPosition((int)(36 * TICK_LENGTH));

        fl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl_motor.setPower(0.5);
        fr_motor.setPower(0.5);
        bl_motor.setPower(0.5);
        br_motor.setPower(0.5);

        moving();

        fl_motor.setPower(0);
        fr_motor.setPower(0);
        bl_motor.setPower(0);
        br_motor.setPower(0);

    }
    void moving(){
        while (fl_motor.isBusy() || fr_motor.isBusy() || bl_motor.isBusy() || br_motor.isBusy()){}
        return;
    }
}