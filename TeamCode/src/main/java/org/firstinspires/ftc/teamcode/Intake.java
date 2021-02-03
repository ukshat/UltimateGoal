package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor intake;

    @Override
    public void runOpMode() {
        // Find each motor on the hardware map
        intake = hardwareMap.dcMotor.get("intake");

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            Gamepad activeGamepad = gamepad1;

            // if the left stick on gamepad 2 is pressed we change the active gamepad to gamepad2
            if(gamepad2.left_stick_button){
                activeGamepad = gamepad2;
            }

            // if the left stick on gamepad 1 is pressed we change the active gamepad to gamepad1
            if(gamepad1.left_stick_button){
                activeGamepad = gamepad1;
            }

            double intakePower = 1;

            // Create variables to hold the direction that the left stick was moved
            double x = activeGamepad.left_stick_x;
            double y = activeGamepad.left_stick_y;

            // if gamepad is equal to a it sets the button for the intake to a
            if(activeGamepad.a){
                intake.setPower(intakePower);
            }

            telemetry.addData("intake wheel motor", intakePower + "\n");
        }

    }
}
