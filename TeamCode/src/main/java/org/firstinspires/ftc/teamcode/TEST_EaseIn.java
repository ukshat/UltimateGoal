package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous (name = "forward 36 inches - easing in")
public class TEST_EaseIn extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotor[4];
    private static final double TICKS_PER_INCH = 34.2795262044082261656;

    @Override
    public void runOpMode() throws InterruptedException {
        //init motors
        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        //set modes & zero power behaviour
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //set left side motors to rotate in opposite direction
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setPower(0.5);
            motors[i].setTargetPosition((int)(36 * TICKS_PER_INCH));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving();

        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    void moving(){
        final int totalTick = motors[0].getTargetPosition();
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            try {Thread.sleep(75);} catch (InterruptedException e) {} //sleep
            int currPos = motors[0].getCurrentPosition();
            double pow = f(currPos, totalTick);
            for(DcMotor motor : motors) motor.setPower(pow);
        }
    }

    double f(int x, int n){
        return -Math.pow((2.8 * Math.pow(x - n / 2, 2)) / (n * n), 2) + 0.5;
    }
}