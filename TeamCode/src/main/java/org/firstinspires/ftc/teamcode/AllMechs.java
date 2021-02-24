package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Intake Assist", group="Linear Opmode")
//@Disabled

public class AllMechs extends LinearOpMode {

    //declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor intake;
    private DcMotor shooter;
    private Servo intakeAssist;

    //@Override
    public void runOpMode() {
        // Find each motor on the hardware map
        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        intakeAssist = hardwareMap.servo.get("IntakeServo");

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            double intakePower = -1;
            double intakeAssistPower = 1;
            double shootingPower = gamepad1.right_trigger;

            // if gamepad is equal to a it sets the button for the intake to a
            if(gamepad1.a){
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0);
            }

            // if gamepad is equal to a it sets the button for the intake to right trigger
            if(shootingPower > 0.5){
                shooter.setPower(1.0);
                intakeAssist.setPosition(intakeAssistPower);
            }
            else {
                shooter.setPower(0);
                intakeAssist.setPosition(0);
            }

            telemetry.addData("intake wheel motor", intakePower + "\n");
            telemetry.addData("shooting wheel motor", shootingPower + "\n");
            telemetry.addData("intake assist servo", intakePower + "\n");
            telemetry.update();
        }

    }
}
