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

        rotate(360);

    }

    void rotate(double degrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        final double startAngle = angle;
        setDirection(degrees < 0 ? 4 : 5, motors);

        while (angle < Math.toRadians(degrees - 2) || angle > Math.toRadians(degrees + 2)){
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = orientation.firstAngle;
            for (DcMotor motor: motors){
                motor.setPower(0.2);
            }
        }
        for (DcMotor motor: motors){
            motor.setPower(0);
        }

    }


    void setDirection(int config, DcMotor[] motors){

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