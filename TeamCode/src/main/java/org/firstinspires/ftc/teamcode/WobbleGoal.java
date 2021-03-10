package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Pot Test")
//@Disabled

public class WobbleGoal extends LinearOpMode {
    Servo release;
    Pot arm = new Pot();

    @Override
    public void runOpMode() throws InterruptedException {
        release = hardwareMap.servo.get("wobbleservo");

        double armPosition = arm.getPosition();
        waitForStart();

        System.out.println(arm.getPosition());

        if(gamepad1.b){
            System.out.println(arm.getPosition());
            arm.setPosition(25);
            if(armPosition == 25){
                release.setPosition(0);
            }

        } else {
            arm.setPosition(armPosition);
            release.setPosition(1);
        }

        System.out.println(arm.getPosition());

        sleep(20000);

        telemetry.addData("intake assist servo", arm + "\n");
        telemetry.update();
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
