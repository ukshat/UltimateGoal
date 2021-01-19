package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "wobble test")
public class TEST_Switch extends LinearOpMode {

    DcMotor wobble;
    Servo claw;
    DigitalChannel limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.digitalChannel.get("wobbleswitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        wobble = hardwareMap.dcMotor.get("wobblemotor");
//        claw = hardwareMap.servo.get("wobbleservo");
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        claw.scaleRange(0, 1);

        waitForStart();

        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobble.setPower(-0.1);
        while(opModeIsActive() && !limitSwitch.getState()) sleep(20);

//        wobble.setTargetPosition((int)(288.0 * 26/10/180 * (162.47-90.0)));
//        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wobble.setPower(0.1);
//        while (wobble.isBusy() && opModeIsActive()){
//            sleep(20);
//        }
        wobble.setPower(0);
//        claw.setPosition(1);
//        while (claw.getPosition() < 0.9){
//            sleep(20);
//        }

    }
}
