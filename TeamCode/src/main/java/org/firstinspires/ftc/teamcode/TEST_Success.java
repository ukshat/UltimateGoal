package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous (name = "forward 36 inches")
public class TEST_Success extends LinearOpMode {

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
            motors[i].setTargetPosition((int)(36 *TICKS_PER_INCH));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        moving(motors, true, telemetry);

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

                telem.addData("Current Power: ", "%.3f",  motors[0].getPower());
                telem.update();

                point -= pointDecrement;
            }
        }
        return;
    }
}