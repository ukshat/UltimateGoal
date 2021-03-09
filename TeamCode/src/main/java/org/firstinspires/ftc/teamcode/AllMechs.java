package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private Servo release;
    private Pot arm;

    //@Override
    public void runOpMode() {
        // Find each motor on the hardware map
        intake = hardwareMap.dcMotor.get("intake");
        shooter = hardwareMap.dcMotor.get("shooter");
        intakeAssist = hardwareMap.servo.get("IntakeServo");
        release = hardwareMap.servo.get("wobbleservo");
        arm = new Pot();

        //wait for driver to press play
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            try {Thread.sleep(25);} catch (InterruptedException e) {} //sleep

            double intakePower = -1;
            double intakeAssistPower = 1;
            double shootingPower = gamepad1.right_trigger;
            double armPosition = arm.getPosition();
            waitForStart();

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

            if(gamepad1.b){
                arm.setPosition(25);
                if(armPosition == 25){
                    release.setPosition(0);
                }

            } else {
                arm.setPosition(armPosition);
                release.setPosition(1);
            }

            telemetry.addData("intake wheel motor", intakePower + "\n");
            telemetry.addData("shooting wheel motor", shootingPower + "\n");
            telemetry.addData("intake assist servo", intakePower + "\n");
            telemetry.update();
        }

    }

    class Pot{

        DcMotorEx motor;
        AnalogInput inp;
        volatile double target;
        volatile boolean shouldMove = true;
        double lowerBound = 0.3, upperBound = 1.35;

        public Pot() {
            motor = (DcMotorEx) hardwareMap.get("wobblemotor");
            inp = hardwareMap.analogInput.get("wobblepot");
        }

        public double getPosition() {
            return (inp.getVoltage() - lowerBound) * 100 / (upperBound - lowerBound);
        }

        public void setPosition(double newTarget){
            telemetry.addData("Target", newTarget);
            telemetry.update();

            sleep(3000);

            if (shouldMove) {
                this.target = newTarget;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        shouldMove = false;
                        if (target > getPosition()) {
                            motor.setVelocity(-100);
                            while (!shouldMove && target > getPosition() && opModeIsActive() && inp.getVoltage() < upperBound){
                                sleep(20);

                                telemetry.addData("Position", getPosition());
                                telemetry.update();
                            }
                        } else {
                            motor.setVelocity(100);
                            while (!shouldMove && target < getPosition() && opModeIsActive() && inp.getVoltage() > lowerBound){
                                sleep(20);

                                telemetry.addData("Position", getPosition());
                                telemetry.update();
                            }
                        }

                        shouldMove = true;

                        motor.setVelocity(0);
                    }
                }).run();
            }
            else{
                shouldMove = true;
                setPosition(newTarget);
            }
        }
    }
}
