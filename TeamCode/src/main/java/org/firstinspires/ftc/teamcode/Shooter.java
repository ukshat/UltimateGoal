package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Intake", group="Linear Opmode")
//@Disabled

public class Shooter extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor shooter;

    //@Override
    public void runOpMode() {
        // Find each motor on the hardware map
        shooter = hardwareMap.dcMotor.get("shooter");

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            double shootingPower = gamepad1.right_trigger;

            // if gamepad is equal to a it sets the button for the intake to a
            if(shootingPower > 0.5){
                shooter.setPower(1.0);
            }
            else{
                shooter.setPower(0);
            }

            telemetry.addData("shooting wheel motor", shootingPower + "\n");
            telemetry.update();
        }

    }
}
