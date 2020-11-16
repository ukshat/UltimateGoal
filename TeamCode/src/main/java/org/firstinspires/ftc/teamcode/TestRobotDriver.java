package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Various move tests")
public class TestRobotDriver extends RobotMotionDriver {
    @Override
    public void runOpMode() throws InterruptedException {

        //Test 1 : Move forward 12 inches
        this.move(12 , 0.25 , DriveDirection.Forward, true);

        Thread.sleep(2000);

        //Test 2 : slide right 12 inches
        this.move(12, 0.25, DriveDirection.SlideRight, true);

        Thread.sleep(2000);

        //Test 3 : move back 12 inches
        this.move(12, 0.25, DriveDirection.Reverse, true);

        Thread.sleep(2000);

        //Test 4 : slide left 12 inches
        this.move(12, 0.25, DriveDirection.SlideLeft, true);

        Thread.sleep(2000);

    }
}