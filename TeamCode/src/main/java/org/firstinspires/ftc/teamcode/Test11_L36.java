package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "left 36")
public class Test11_L36 extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotor[4];
    byte[] motorDirs = new byte[4];

    static final double TILE_LENGTH = 23.5;
    static final double TICK_LENGTH = 100 / Math.PI;

    @Override
    public void runOpMode() throws InterruptedException {
        //init motors
        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        //default directions
        motorDirs = Util.setDefaultDirs(motorDirs);

        //set modes & zero power behaviour
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();

        for(int i = 0; i < 4; i++){
            motors[i].setPower(0.5);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition((int)(motorDirs[i] * 18 * Util.TICKS_PER_INCH * ((i % 3 == 0) ? -1 : 1)));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        Util.moving(motors);

        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }
}