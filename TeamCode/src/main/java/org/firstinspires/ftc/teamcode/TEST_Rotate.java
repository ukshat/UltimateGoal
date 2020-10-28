package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "rotate 10 times")
public class TEST_Rotate extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotor[4];

    BNO055IMU imu;
    BNO055IMU.Parameters params;
    static final double TICKS_PER_INCH = 34.2795262044082261656;
    static final double ARC_LENGTH = 9.487480983 * (5.0 / 4) * (11.0/10) * (5/5.25);
    static Telemetry telem;

    @Override
    public void runOpMode() throws InterruptedException {
        //init motors
        imu = (BNO055IMU) hardwareMap.get("imu");

        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        telem = telemetry;
        waitForStart();

        rotate(360, 0.5, motors, (byte)0);

    }

    static void rotate(double degrees, double power, DcMotor[] motors, byte config){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //distance that each wheel will travel in rotation type movement
        double arcLength =  ARC_LENGTH * radians;

        setDirection((byte)(config + 4), motors);

        //set motor distances and powers
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition((int)(arcLength * TICKS_PER_INCH));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(power);
        }

        moving(motors, true, telem);

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    static void moving(DcMotor[] motors, boolean slowDown, Telemetry telem){

        final int totalTick = motors[0].getTargetPosition();
        final int decrements = 20;
        final double ratio = 7.0/12;
        int point = (int)((1 - ratio) * totalTick);
        final double  decrement = motors[0].getPower() / 21;
        final int pointDecrement = point/decrements + 1;
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            int ticksLeft = totalTick - motors[0].getCurrentPosition();
            if (ticksLeft == point && slowDown && motors[0].getPower() > 0.08){
                motors[0].setPower(motors[0].getPower() - decrement);
                motors[1].setPower(motors[0].getPower() - decrement);
                motors[2].setPower(motors[0].getPower() - decrement);
                motors[3].setPower(motors[0].getPower() - decrement);

                telem.addData("Current Power: ", motors[0].getPower());
                telem.update();

                point -= pointDecrement;
            }
        }
        return;
    }

    static void setDirection(byte config, DcMotor[] motors){

        for(int i = 0; i < 4; i++){
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            if (config == 0 && i % 2 == 0) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 1 && i <=1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 2 && i % 2 == 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 3 && i >= 2) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 4) motors[i].setDirection(DcMotor.Direction.REVERSE);
        }
    }
}