package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TEST_UsingPot extends LinearOpMode {
    DcMotorEx wobble;
    Pot pot;
    @Override
    public void runOpMode() throws InterruptedException {
        wobble = (DcMotorEx) hardwareMap.get("wobblemotor");
        pot = new Pot ();
        waitForStart();
        pot.setPosition(5);
    }

    class Pot{
        DcMotorEx motor;
        AnalogInput inp;
        volatile double target;
        volatile boolean moving;
        double lowerBound = 0, upperBound = 3.3;
        public Pot() {
            motor = (DcMotorEx) hardwareMap.get("wobblemotor");
            inp = hardwareMap.analogInput.get("");
        }
        public double getPosition() {
            return (inp.getVoltage() - lowerBound) * 100 / (upperBound - lowerBound);

        }
        public void setPosition(double newTarget){
            this.target = newTarget;
            if (newTarget >= 0 && newTarget <= 100 && newTarget != target && ! moving)new Thread(new Runnable(){

                @Override
                public void run() {
                    moving = true;
                    if (target > getPosition()){
                        motor.setVelocity(400);
                        while (target > getPosition() && opModeIsActive() &&  getPosition() < upperBound ) sleep(20);
                    } else{
                        motor.setVelocity(-400);
                        while (target < getPosition() && opModeIsActive() &&  getPosition() > lowerBound ) sleep(20);
                    }

                    moving = false;
                }
            }).run();

        }
    }
}
