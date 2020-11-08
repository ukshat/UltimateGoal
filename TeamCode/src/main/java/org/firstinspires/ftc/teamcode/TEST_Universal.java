package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Test for all permutations of methods ")
public class TEST_Universal extends LinearOpMode {

    static final double TICKS_PER_INCH = 34.2795262044082261656;
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotor[4];

    BNO055IMU imu;
    BNO055IMU.Parameters params;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = (BNO055IMU) hardwareMap.get("imu");
        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        for (DcMotor motor : motors)motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        waitForStart();

        move(0,40,0.5,motors);
        sleep(1000);
        rotate(-90, motors, imu);
        rotate(-90, motors, imu);
        sleep(1000);
        rotate(90, motors, imu);
        sleep(1000);
        move(1,40,0.5, motors);
        sleep(1000);
        rotate(90, motors, imu);
    }

    static void move(int config, double distance, double speed, DcMotor[] motors){
        setDirection(config, motors);
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setPower(speed);
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
        final int totalTick = motors[0].getTargetPosition();

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){
            try {Thread.sleep(75);} catch (InterruptedException e) {} //sleep
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


    void rotate(double degrees, DcMotor[] motors, BNO055IMU imu){

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        final double startAngle = angle;
        for(int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (degrees < 0){
            setDirection(4, motors );
            while (angle > degrees * 0.925 && opModeIsActive()){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = orientation.firstAngle;
                for(int i = 0; i < 4; i++) {
                    motors[i].setPower(0.2);
                }
                sleep(20);
            }

        }

        else {
            setDirection(5, motors);
            while (angle < degrees * 0.925){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angle = orientation.firstAngle;
                for (int i = 0; i < 4; i++){
                    motors[i].setPower(0.2);
                }
                sleep(20);
            }

        }
        for (DcMotor motor: motors){
            motor.setPower(0);
        }


    }

    static void setDirection(int config, DcMotor[] motors){

        for(int i = 0; i < 4; i++){
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            if (config == 0 && i % 2 == 0) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 1 && i >= 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 2 && i % 2 == 1) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 3 && i <= 3) motors[i].setDirection(DcMotor.Direction.REVERSE);
            else if (config == 4) motors[i].setDirection(DcMotor.Direction.REVERSE);
        }
    }

}