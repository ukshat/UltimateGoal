package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.factory.RobotFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import javax.xml.crypto.NoSuchMechanismException;

import sun.rmi.runtime.Log;

/**
 * Class that is provides all basic motion control
 * methods
 */
public class AutonPID extends LinearOpMode {

    static final double TILE_LENGTH = 23.5;

    // number of ticks in one inch
    static final double TICKS_PER_INCH = 34.2795262044082261656;

    // constant for offset between front/back to left/right
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;

    // stores the current direction of the robot
    double currOrientation;

    DcMotorEx[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotorEx[4];



    @Override
    public void runOpMode() throws InterruptedException {
        // init motors
        motors[0] = (DcMotorEx) hardwareMap.dcMotor.get("LeftFront");
        motors[1] = (DcMotorEx) hardwareMap.dcMotor.get("RightFront");
        motors[2] = (DcMotorEx) hardwareMap.dcMotor.get("LeftRear");
        motors[3] = (DcMotorEx) hardwareMap.dcMotor.get("RightRear");

        for (DcMotor motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients desiredPidCoefficients = getPIDCoefficients();
        telemetry.addLine();
        telemetry.addData("Final PID Coeffs" , "P=" + desiredPidCoefficients.p +
                ", I=" + desiredPidCoefficients.i + ", D=" + desiredPidCoefficients.d);


    }

    public PIDFCoefficients getPIDCoefficients() throws InterruptedException {
        /*
         * We will move forward 36" and back 36".
         * We will do this 50 times each...meaning, 100 iterations
         * Everytime, after the robot stops, we will check
         * the position. We will compare the actual position with the
         * desired position. If error is positive then we will reduce the
         * PID constants and try again. If error is negative then we will increase
         * the PID constants and try again.
         * In 100 trials we will pick the PID constants where we get minimum error
         * */
        double distanceInInches=36.0;
        double ticksToMove = distanceInInches * TICKS_PER_INCH;

        double minimumError = Double.MAX_VALUE;
        PIDFCoefficients desiredPid = null;
        DcMotorEx motorEx = null;
        motorEx = (DcMotorEx)frontLeftMotor;
        motorEx.setTargetPositionTolerance(0);
        desiredPid = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine();
        telemetry.addData("Starting PID Coeffs" , "P=" + desiredPid.p +
                ", I=" + desiredPid.i + ", D=" + desiredPid.d);

        for (int loop = 0  ; loop < 1 ; loop++){
            //setting the tolerance to 0, to be as exact as possible


            move(2 *(loop % 2), 36,0.5);
            double actualPosition = motorEx.getCurrentPosition();

            double error = actualPosition - ticksToMove;
            PIDFCoefficients pidCoef = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            if(Math.abs(error) < Math.abs (minimumError)){
                minimumError = error;
                desiredPid = pidCoef;
            }

            if (error > 0){
                pidCoef.p = pidCoef.p * 0.99;
                pidCoef.i = pidCoef.i * 0.99;
                pidCoef.d = pidCoef.d * 0.99;
            }else{
                pidCoef.p = pidCoef.p * 1.01;
                pidCoef.i = pidCoef.i * 1.01;
                pidCoef.d = pidCoef.d * 1.01;
            }
            pidCoef.f = 0;//To ensure we use basic PID control
            motorEx.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoef);
            sleep(10000);
        }

        return desiredPid;
    }

    void setDirection(int config){

        for(int i = 0; i < 4; i++){
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            if (config == 0 && i % 2 == 0) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 1 && i >= 2) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 2 && i % 2 == 1) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 3 && i <= 1) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            } else if (config == 4) {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            }
        }

    }

    void move(int config, double distance, double speed){
        setDirection(config);
        for(int i = 0; i < 4; i++){
            // reset encoders
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // set power to motors
            motors[i].setPower(speed);
            // set target position
            // if left/right, multiply by horizontal strafe constant
            motors[i].setTargetPosition((int)(distance * TICKS_PER_INCH * (config % 2 == 1 ? HORIZONTAL_STRAFE : 1)));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving(motors, true);

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    static void moving(DcMotor[] motors, boolean slowDown){
        // stores the total distance to move
        final int totalTick = motors[0].getTargetPosition();

        // continue the while loop until all motors complete movement
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            // delay
            try {Thread.sleep(75);} catch (InterruptedException e) {} //sleep
            // if we choose to, increment speed gradually to minimize jerking motion
            if(slowDown){
                int currPos = motors[0].getCurrentPosition();
                double pow = f(currPos, totalTick);
                for(DcMotor motor : motors) motor.setPower(pow);
            }
        }
    }

    static double f(int x, int n){
        return -Math.pow((2.8 * Math.pow(x - n / 2, 2)) / (n * n), 2) + 0.5;
    }
}