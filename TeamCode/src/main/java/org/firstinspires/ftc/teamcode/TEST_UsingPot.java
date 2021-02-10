package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Pot Test")
public class TEST_UsingPot extends LinearOpMode {
    DcMotorEx wobble;
    Pot pot;
    @Override
    public void runOpMode() throws InterruptedException {
        wobble = (DcMotorEx) hardwareMap.get("wobblemotor");
        pot = new Pot ();
        waitForStart();
        pot.setPosition(95);
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
            if (shouldMove) {
                this.target = newTarget;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        shouldMove = false;
                        if (target > getPosition()) {
                            motor.setVelocity(100);
                            while (!shouldMove && target > getPosition() && opModeIsActive() && inp.getVoltage() < upperBound) sleep(20);
                        } else {
                            motor.setVelocity(-100);
                            while (!shouldMove && target < getPosition() && opModeIsActive() && inp.getVoltage() > lowerBound) sleep(20);
                        }

                        shouldMove = true;
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
