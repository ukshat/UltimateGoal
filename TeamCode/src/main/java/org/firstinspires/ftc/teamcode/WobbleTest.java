package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "wobble test")
public class WobbleTest extends LinearOpMode {

    DcMotor wobble;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        wobble = hardwareMap.dcMotor.get("");
        claw = hardwareMap.servo.get("");
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobble.setTargetPosition((int)(288.0 * 26/10/180 * (162.47-90.0)));
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble.setPower(0.1);
        while (wobble.isBusy() && opModeIsActive()){
            sleep(20);
        }
        wobble.setPower(0);
        claw.setPosition(1);

    }
}
