package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import javax.crypto.Mac;

@TeleOp(name = "Pot Test")
//@Disabled

public class WobbleGoal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        WobbleMech wobbleMech = new WobbleMech();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            if(gamepad1.b){
                wobbleMech.setPosition(wobbleMech.lowerBound);
            }
            if(gamepad1.x){
                wobbleMech.setPosition(wobbleMech.upperBound);
            }
            if(gamepad1.y){
                wobbleMech.open();
            }
            if(gamepad1.a){
                wobbleMech.close();
            }

        }

    }

    class WobbleMech {
        private final Servo clasp;
        private final DcMotorEx arm;
        private final AnalogInput inp;
        private boolean isClosed;
        final double safeBound = 0.62, lowerBound = 1.0, upperBound = 2.21;

        public WobbleMech() {
            clasp = hardwareMap.servo.get("wobbleservo");
            clasp.setDirection(Servo.Direction.FORWARD);
            arm = (DcMotorEx) hardwareMap.get("wobblemotor");
            inp = hardwareMap.analogInput.get("wobblepot");
        }

        public boolean isClosed (){
            return isClosed;

        }

        public void close (){
            clasp.setPosition(0);
            isClosed = true;
        }

        public void open (){
            clasp.setPosition(1);
            isClosed = false;
        }

        private void setPosition(double voltage) {
           if(voltage > inp.getVoltage()){
               arm.setVelocity(-100);
               while(inp.getVoltage() < upperBound){
                   sleep(20);
               }
               arm.setVelocity(0);
           }
           else {
               arm.setVelocity(100);
               while(inp.getVoltage() > lowerBound){
                   sleep(20);
               }
               arm.setVelocity(0);
           }

        }
    }

}

