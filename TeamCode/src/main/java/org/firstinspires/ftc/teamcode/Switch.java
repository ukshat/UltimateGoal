package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous (name = "switch test")
public class Switch extends LinearOpMode {

    Servo claw;
    TouchSensor limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.touchSensor.get("wobbleswitch");

        waitForStart();

        while(opModeIsActive()) {
            sleep(20);
            if (limitSwitch.isPressed()){
                telemetry.addLine("Switch is pressed.");
            } else {
                telemetry.addLine("Switch is not pressed.");
            }
            telemetry.update();
        }

    }
}
