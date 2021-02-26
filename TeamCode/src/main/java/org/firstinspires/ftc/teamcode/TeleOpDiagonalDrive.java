package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Basic: Diagonal Drive", group="Linear Opmode")
//@Disabled


public class TeleOpDiagonalDrive extends LinearOpMode {

    // number of ticks in one inch
    static final double TICKS_PER_INCH = 34.2795262044082261656;

    // constant for offset between front/back to left/right
    static final double HORIZONTAL_STRAFE = 36.0 / 30.75;

    DcMotorEx[/*Front Left, Front Right, Back Left, Back Right*/] motors = new DcMotorEx[4];

    @Override
    public void runOpMode() throws InterruptedException {

        motors[0] = (DcMotorEx) hardwareMap.dcMotor.get("LeftFront");
        motors[1] = (DcMotorEx) hardwareMap.dcMotor.get("RightFront");
        motors[2] = (DcMotorEx) hardwareMap.dcMotor.get("LeftRear");
        motors[3] = (DcMotorEx) hardwareMap.dcMotor.get("RightRear");

        int w = 34;
        int h = 80;
        double distance = Math.hypot(w, h);
        double speed = 0.5;

        if(gamepad1.b) {
            double degrees = Math.atan2(h, w) - 90;
            move(degrees, distance, speed);
        }

        if(gamepad1.x) {
            double degrees = 90 - Math.atan2(h, w);
            move(degrees, distance, speed);
        }

    }

    double map(double from) { return from / HORIZONTAL_STRAFE; }

    void move(double deg, double distance, double speed) {
        setDirection(0);
        //inch to ticks
        distance *= TICKS_PER_INCH;

        //deg to rad
        deg = Math.toRadians(deg);

        //x and y of mapped point on ellipse
        double x = Math.cos(deg) * distance, y = map(Math.sin(deg)) * distance;

        //rotate axis about origin by 45 deg ccw
        deg = Math.asin(y / distance) + Math.PI / 4;

        //rewrite x and y to be mapped to new axis
        x = Math.cos(deg) * distance;
        y = map(Math.sin(deg)) * distance;

        /*Front Left, Front Right, Back Left, Back Right*/
        for (int i = 0; i < 4 && opModeIsActive(); i++)
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //FL
        motors[0].setTargetPosition((int) y);
        //FR
        motors[1].setTargetPosition((int) x);
        //BL
        motors[2].setTargetPosition((int) x);
        //BR
        motors[3].setTargetPosition((int) y);

        for (int i = 0; i < 4 && opModeIsActive(); i++)
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set x and y to be between -0.5 and 0.5
        double max = Math.max(Math.abs(x), Math.abs(y));
        x = x / max * speed;
        y = y / max * speed;

        while ((motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()) && opModeIsActive() && !gamepad1.y) {
            // delay
            sleep(75);

            //current position of motor travel
            int currPosX = motors[1].getCurrentPosition();
            int currPosY = motors[0].getCurrentPosition();

            //calculate ticks per second
            double xPow = fWithMaxPow(currPosX, (int) distance, x) * 40 * TICKS_PER_INCH;
            double yPow = fWithMaxPow(currPosY, (int) distance, y) * 40 * TICKS_PER_INCH;

            //FL
            motors[0].setVelocity((int) yPow);
            //FR
            motors[1].setVelocity((int) xPow);
            //BL
            motors[2].setVelocity((int) xPow);
            //BR
            motors[3].setVelocity((int) yPow);
        }

        for (DcMotorEx m : motors) m.setVelocity(0);
    }

    static double fWithMaxPow(int x, int n, double maxPow){
        return maxPow * (1 - Math.pow(3.85 * Math.pow(x - n / 2, 2) / (n * n), 1.75));
    }

    void setDirection(int config) {

        for (int i = 0; i < 4 && opModeIsActive(); i++) {
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
}