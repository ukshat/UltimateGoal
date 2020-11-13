package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.factory.RobotFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import javax.xml.crypto.NoSuchMechanismException;

/**
 * Class that is provides all basic motion control
 * methods
 */
public class AutonPID extends RobotMotionDriver {

    @Override
    public void runOpMode() throws InterruptedException {
        /**TODO...how to log the values to see the PID coefficients*/
        PIDFCoefficients desiredPidCoefficients = getPIDCoefficients();
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

        for (int loop = 0 ; loop < 10 ; loop++){
            if(frontLeftMotor instanceof DcMotorEx /*Just checking one motor is good enough*/){
                //setting the tolerance to 0, to be as exact as possible
                DcMotorEx motorEx = (DcMotorEx)frontLeftMotor;
                motorEx.setTargetPositionTolerance(0);
                DriveDirection dir = DriveDirection.Forward;
                if (loop % 2 == 0)
                    dir = DriveDirection.Forward;
                else
                    dir = DriveDirection.Reverse;

                move(distanceInInches,0.5,dir, true);
                double actualPosition = motorEx.getCurrentPosition();

                double error = actualPosition - ticksToMove;
                PIDFCoefficients pidCoef = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

                if(error < minimumError){
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
            }else{
                /*We cannot use this method*/
                throw new NoSuchMechanismException();
            }
        }

        return desiredPid;
    }
}