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
    }

    private void innerBotRun(byte r /* input -1 for left side, +1 for right side*/){
        move(0, 36, 0.5); //move to stack

        int rings = getStack(r);

        move(0, 24, 0.5); //move to launch range

        rotate(-3.576334375 * r, 0.5);
        launch((byte)1);

        rotate(-(9.462322208 - 3.576334375) * r, 0.5);
        launch((byte)2);

        rotate(-(15.15406805 - 9.462322208) * r, 0.5);
        launch((byte)3);

        rotate(15.15406805 * r, 0.5);

        switch (rings){
            case 0:
                //LONG ROUTE
                break;
            case 1:
                move(0, 36, 0.5);
                break;
            case 4:
                //LONG ROUTE
                break;
            default:
                move(0, 36, 0.5);
                break;
        }
        dropWobbleGoal();
        switch (rings) {
            case 0:
                //LONG ROUTE
                break;
            case 1:
                move(180, 24, 0.5);
                break;
            case 4:
                //LONG ROUTE
                break;
            default:
                move(180, 24, 0.5);
                break;
        }
    }

    private void outerBotRun(byte r /* input -1 for left side, +1 for right side*/){
        move(0, 36, 0.5); //move to stack

        int rings = getStack(r);

        move(0, 24, 0.5); //move to launch range

        rotate(-21.59531045 * r, 0.5);
        launch((byte)1);

        rotate(-(26.56505118 - 21.59531045) * r, 0.5);
        launch((byte)2);

        rotate(-(31.13897244 - 26.56505118) * r, 0.5);
        launch((byte)3);

        rotate(31.13897244 * r, 0.5);

        switch (rings){
            case 0:
                move(0, 12, 0.5);
                break;
            case 1:
                move(0, 36, 0.5);
                break;
            case 4:
                move(0, 60, 0.5);
                break;
            default:
                move(0, 12, 0.5);
                break;
        }
        dropWobbleGoal();
        switch (rings) {
            case 1:
                move(180, 24, 0.5);
                break;
            case 4:
                move(180, 48, 0.5);
                break;
        }
    }

    private void dropWobbleGoal(){}

    private void launch(byte target){}

    private int getStack(byte r){return 0;}

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
