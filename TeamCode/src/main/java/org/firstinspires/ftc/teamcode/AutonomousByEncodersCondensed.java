package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonomousByEncodersCondensed extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors;

    static final double TICK_LENGTH = 0.22165663;

    @Override
    public void runOpMode() throws InterruptedException {
        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // 304.8 mm is how much we want to move
        // 0.22165663 mm is the length of one tick

        //old code
        double distance = 304.8;
        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(distance / TICK_LENGTH));
        }
        motors[0].setPower(-1);
        motors[1].setPower(1);
        motors[2].setPower(1);
        motors[3].setPower(-1);
        moving();
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

        //new Code
        {//RIGHT_Right
            move(0, 36, 0.5); //move to stack

            int rings = getStack();

            move(0, 24, 0.5); //move to stack

            //power shots

            switch (rings){
                case 0:
                    move(0, 36, 0.5);
                    break;
                case 1:
                    move(68.19859, 64.6, 0.5);
                    break;
                case 4:
                    move(0, 3.5 * 24, 0.5);
                    break;
            }
            dropWobbleGoal();
        }

    }

    private void dropWobbleGoal(){}

    private int getStack(){return 0;}

    void rotate(double degrees, double power){
        double radians = degrees * Math.PI / 180;
        double[] pows = {power, -power, power, -power};
        if (degrees < 0){
            for(int i = 0; i < 4; i++){
                pows[i] *= -1;
            }
        }

        double arcLength =  9.487480983 * radians;

        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(arcLength / TICK_LENGTH));
            motors[i].setPower(pows[i]);
        }

        moving();

        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    void move(double degrees, double distance, double power){
        double radians = degrees * Math.PI / 180;
        double x = power * Math.cos(radians);
        double y = power * Math.sin(radians);
        double[] pows = FinalDriveCode.calculateMotorPower(x, y, 0);

        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(distance / TICK_LENGTH));
            motors[i].setPower(pows[i]);
        }

        moving();

        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    void moving(){
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){}
        return;
    }
}
