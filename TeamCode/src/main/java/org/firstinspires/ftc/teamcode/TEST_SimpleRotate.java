package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "simple rotate 90 degrees")
public class TEST_SimpleRotate extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotor[4];
    BNO055IMU imu;
    BNO055IMU.Parameters params;

    @Override
    public void runOpMode() throws InterruptedException {
        //init motors
        imu = (BNO055IMU) hardwareMap.get("imu");

        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD);

        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        waitForStart();

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        final double startAngle = angle;
        //orientation.toAxesReference(AxesReference.INTRINSIC);

        while (angle < 180 * 0.9236 && opModeIsActive()){
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = orientation.firstAngle;
            for (int i = 0; i < motors.length; i++){
                motors[i].setPower(0.2);
            }
            sleep(20);
        }
        telemetry.addData("Start: ", startAngle + "\n");
        telemetry.addData("Finish: ", angle);
        telemetry.update();
        for (DcMotor motor: motors){
            motor.setPower(0);
        }

    }
}
