package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class RobotMovesDriver extends LinearOpMode{
    public enum DriveDirection{
        Forward,
        Reverse,
        SlideLeft,
        SlideRight
    }
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    private final double TICKS_PER_INCH = 107/Math.PI;

    /**
     * Class constructor
     */
    public RobotMovesDriver(){
        frontLeftMotor = hardwareMap.dcMotor.get("fl_motor");
        frontRightMotor= hardwareMap.dcMotor.get("fr_motor");
        backLeftMotor = hardwareMap.dcMotor.get("bl_motor");
        backRightMotor= hardwareMap.dcMotor.get("br_motor");
    }

    /**
     * Method that makes the robot move forward
     * @param distanceInInches  How many inches to move forward
     * @param power A number between -1.0 and +1.0
     * @param driveDirection One of four values forward,reverse,left and right
     * @param runAutonomous If true, then do not wait for controller start button
     */
    public void move(double distanceInInches , double power,
                            DriveDirection driveDirection,
                            boolean runAutonomous)
            throws IllegalArgumentException{
        if(distanceInInches <= 0) throw new IllegalArgumentException("Distance must be a positive number");
        if(power < -1 || power > 1) throw new IllegalArgumentException("Power must be between -1 and +1");

        /**
         * Motor has Mecanum wheels...
         * To move forward all 4 wheels must move forward direction..
         */

        //Stop motors first...
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        //Reset encoders to zero
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch(driveDirection){
            case Forward:
                frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
                backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                backRightMotor.setDirection(DcMotor.Direction.FORWARD);
                break;
            case Reverse:
                frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
                backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                backRightMotor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case SlideLeft:

                frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
                backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                backRightMotor.setDirection(DcMotor.Direction.REVERSE);
                break;

            case SlideRight:
                frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
                backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                backRightMotor.setDirection(DcMotor.Direction.FORWARD);
                break;
        }


        int ticksToCountForDistance = (int)(distanceInInches * TICKS_PER_INCH);

        //set ticks for each motor...
        frontLeftMotor.setTargetPosition(ticksToCountForDistance);
        frontRightMotor.setTargetPosition(ticksToCountForDistance);
        backLeftMotor.setTargetPosition(ticksToCountForDistance);
        backRightMotor.setTargetPosition(ticksToCountForDistance);

        //Now set the mode to tell motors to run until ticks elapsed
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Now wait for controller Start button...
        if(runAutonomous == false) waitForStart();

        //Set the power of the motor...this will start them..
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        //Now wait for motors to reach their destination...
        while(opModeIsActive()){
            if(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())
                //Yield control of the thread..
                try{ Thread.sleep(100);} catch (InterruptedException ex){}
        }
        //ask motors to stop fully...
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}