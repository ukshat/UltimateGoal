package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonomousByEncodersCondensed extends LinearOpMode {

    DcMotor[/*Front Left, Front Right, Back Left, Back Right*/] motors;

    static final double TILE_LENGTH = 23.5;
    static final double TICK_LENGTH = 0.22165663;

    @Override
    public void runOpMode() throws InterruptedException {
        //init motors
        motors[0] = hardwareMap.dcMotor.get("LeftFront");
        motors[1] = hardwareMap.dcMotor.get("RightFront");
        motors[2] = hardwareMap.dcMotor.get("LeftRear");
        motors[3] = hardwareMap.dcMotor.get("RightRear");

        //set modes & zero power behaviour
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //set left side motors to rotate in opposite direction
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
    }

    /**
     * executes movement around the game field if robots are in positions 2 or 3
     *
     * @params r either -1 or 1 to determine which side of the game field robot is in
     * */
    private void innerBotRun(byte r /* input -1 for left side, +1 for right side*/){
        //move to stack
        move(0, TILE_LENGTH * 1.5, 0.5);

        //get ring stack height
        int rings = getStack(r);

        //move to launch range
        move(0, TILE_LENGTH, 0.5);

        launchAllAutonRings((byte)(1 * r));

        switch (rings){
            case 0:
                //if there are 0 rings, execute this path
                //LONG ROUTE
                break;
            case 1:
                //if there is 1 ring, execute this path
                move(0, TILE_LENGTH * 1.5, 0.5);
                break;
            case 4:
                //if there are 4 rings, execute this path
                //LONG ROUTE
                break;
        }

        dropWobbleGoal();

        switch (rings) {
            case 0:
                //if there are 0 rings, execute this path
                //LONG ROUTE
                break;
            case 1:
                //if there is 1 ring, execute this path
                move(180, TILE_LENGTH, 0.5);
                break;
            case 4:
                //if there are 4 rings, execute this path
                //LONG ROUTE
                break;
        }
    }

    /**
     * executes movement around the game field if robots are in positions 1 or 4
     *
     * @params r either -1 or 1 to determine which side of the game field robot is in
     * */
    private void outerBotRun(byte r /* input -1 for left side, +1 for right side*/){
        //move to stack
        move(0, TILE_LENGTH * 1.5, 0.5);

        //get height of stack of rings
        int rings = getStack(r);

        //move to launch range
        move(0, TILE_LENGTH, 0.5);

        launchAllAutonRings((byte)(1 * r));

        switch (rings){
            case 0:
                //if there are 0 rings, execute this path
                move(0, TILE_LENGTH * 0.5, 0.5);
                break;
            case 1:
                //if there is 1 ring, execute this path
                move(0, TILE_LENGTH * 1.5, 0.5);
                break;
            case 4:
                //if there are 4 rings, execute this path
                move(0, TILE_LENGTH * 2.5, 0.5);
                break;
        }

        dropWobbleGoal();

        switch (rings) {
            case 1:
                //if there are 0 rings, execute this path
                move(180, TILE_LENGTH, 0.5);
                break;
            case 4:
                //if there are 4 rings, execute this path
                move(180, TILE_LENGTH * 2, 0.5);
                break;
        }
    }

    private void dropWobbleGoal(){}

    private void launch(byte target){}

    private int getStack(byte r){return 0;}

    /**
     * Launches all 3 rings for autonomous period
     *
     * @params condition -4 for position 1 powershot, -3 for position 1 top goal, -2 for position 2 powershot, -1 for position 2 top goal
     *                   +4 for position 4 powershot, +3 for position 4 top goal, +2 for position 3 powershot, +1 for position 3 top goal
     * */
    private void launchAllAutonRings(byte condition){
        //either 1 or -1 depending on which side of the field to shoot
        byte side = (byte)(condition / Math.abs((int)condition));

        //depending on each scenario: aim and launch all 3 rings and return to original position
        switch(condition / side){
            case(4):
                rotate(-(21.66115082 - 00.00000000) * side, 0.5);
                launch((byte)1);

                rotate(-(26.72736173 - 21.66115082) * side, 0.5);
                launch((byte)2);

                rotate(-(31.38022942 - 26.72736173) * side, 0.5);
                launch((byte)3);

                rotate(31.38022942 * side, 0.5);

                break;
            case(3):
                rotate(-9.462322208 * side, 0.5);

                for(int i = 0; i < 3; i++) launch((byte) 0);

                rotate(9.462322208 * side, 0.5);

                break;
            case(2):
                rotate(-(3.65222278  - 0.00000000) * side, 0.5);
                launch((byte)1);

                rotate(-(9.65989307  - 3.65222278) * side, 0.5);
                launch((byte)2);

                rotate(-(15.46121774 - 9.65989307) * side, 0.5);
                launch((byte)3);

                rotate(15.46121774 * side, 0.5);

                break;
            case(1):
                rotate(9.462322208 * side, 0.5);

                for(int i = 0; i < 3; i++) launch((byte) 0);

                rotate(-9.462322208 * side, 0.5);

                break;
        }
    }

    /**
     * rotates the robot a certain amount of degrees at a given speed
     *
     * @params degrees amount in degrees to rotate the robot (must be between -180 and 180)
     * @params power speed at which motors will run -- affects speed of rotation
     * */
    void rotate(double degrees, double power){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //motor powers for rotation will always be like this
        double[] pows = {power, -power, power, -power};

        //if turning counter clockwise, reverse motor powers
        if (degrees < 0){
            for(int i = 0; i < 4; i++){
                pows[i] *= -1;
            }
        }

        //distance that each wheel will travel in rotation type movement
        double arcLength =  9.487480983 * radians;

        //set motor distances and powers
        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(arcLength / TICK_LENGTH));
            motors[i].setPower(pows[i]);
        }

        moving();

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    /**
     * moves the robot in a given linear direction for a given distance and speed
     *
     * @params degrees linear direction to move the robot
     * @params distance distance at which to move robot
     * @params power speed at which motors will run -- affects speed of movement
     * */
    void move(double degrees, double distance, double power){
        //convert degrees to radians
        double radians = degrees * Math.PI / 180;

        //calculate dimensions of tentative unit right triangle described by given parameters
        double x = power * Math.cos(radians);
        double y = power * Math.sin(radians);

        //calculate motor powers
        double[] pows = FinalDriveCode.calculateMotorPower(x, y, 0);

        //assign powers and distances to motors
        for(int i = 0; i < 4; i++){
            motors[i].setTargetPosition((int)(distance / TICK_LENGTH));
            motors[i].setPower(pows[i]);
        }

        moving();

        //stop motors
        for(int i = 0; i < 4; i++){
            motors[i].setPower(0);
        }
    }

    void moving(){
        //runs infinite loop until motor stops moving
        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()){}
        return;
    }
}
