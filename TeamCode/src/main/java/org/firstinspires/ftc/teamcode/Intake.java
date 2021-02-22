package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Intake", group="Linear Opmode")
//@Disabled

public class Intake extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor intake;

    //@Override
    public void runOpMode() {
        // Find each motor on the hardware map
        intake = hardwareMap.dcMotor.get("intake");

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            double intakePower = -1;

            // if gamepad is equal to a it sets the button for the intake to a
            if(gamepad1.a){
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0);
            }

            telemetry.addData("intake wheel motor", intakePower + "\n");
            telemetry.update();
        }

    }
}
