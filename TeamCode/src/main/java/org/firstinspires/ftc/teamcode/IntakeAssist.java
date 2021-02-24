package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Intake Assist", group="Linear Opmode")
//@Disabled

public class IntakeAssist extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private Servo intakeAssist;

    //@Override
    public void runOpMode() {
        // Find each motor on the hardware map
        intakeAssist = hardwareMap.servo.get("IntakeServo");

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            double intakePower = 1;
            intakeAssist.setPosition(1);

            // if gamepad is equal to a it sets the button for the intake to a

            /*
            if(gamepad1.b){
                intakeAssist.setPosition(intakePower);
            }
            else {
                intakeAssist.setPosition(0);
            }

            */


            telemetry.addData("intake assist servo", intakePower + "\n");
            telemetry.update();
        }

    }
}
