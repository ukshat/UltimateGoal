package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Pot Test")
public class TEST_WobbleMech extends LinearOpMode {
    WobbleMech wobbleMech;
    @Override
    public void runOpMode() throws InterruptedException {
        wobbleMech = new WobbleMech();
        waitForStart();
        wobbleMech.close();
        sleep(1000);
        wobbleMech.open();

        sleep(20000);
    }

    class WobbleMech {

        private Servo servo;

        private DcMotorEx motor;
        private AnalogInput inp;
        private volatile double target;
        private volatile boolean shouldMove = true;
        private boolean isClosed;
        final double lowerBound = 0.3, upperBound = 1.35;

        public WobbleMech() {

            servo = hardwareMap.servo.get("wobbleservo");
            servo.scaleRange(0,1);
            servo.setDirection(Servo.Direction.FORWARD);
            motor = (DcMotorEx) hardwareMap.get("wobblemotor");
            inp = hardwareMap.analogInput.get("wobblepot");
        }

        public boolean isClosed (){

            return isClosed;

        }

        public void close (){

            servo.setPosition(1);
            isClosed = true;
        }

        public void open (){

            servo.setPosition(0);
            isClosed = false;
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
