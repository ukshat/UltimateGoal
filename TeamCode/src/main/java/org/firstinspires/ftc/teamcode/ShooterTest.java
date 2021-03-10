package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter();
        shooter.shoot();
    }

    class Shooter {

        private final DcMotorEx shooter, intake;
        private final Servo stick;


        public Shooter() {

            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            stick = hardwareMap.servo.get("IntakeServo");
            intake = hardwareMap.get(DcMotorEx.class, "intake");

        }

        public void push() {
            stick.setPosition(1);
        }

        public void pull() {
            stick.setPosition(0);
        }

        public void shoot() {
            shooter.setVelocity(1300);
            push();
            sleep(1500);
            pull();
            intake.setVelocity(-1300);
            push();
            sleep(1500);
            pull();
            intake.setVelocity(0);
            shooter.setVelocity(0);
        }

    }

}
