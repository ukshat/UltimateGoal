package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class TEST_Pot extends LinearOpMode {
    AnalogInput pot;
    @Override
    public void runOpMode() throws InterruptedException {
        pot = hardwareMap.analogInput.get("");
        waitForStart();
        while(opModeIsActive()){
            sleep(20);
            telemetry.addData("Potentiometer", pot.getVoltage());
            telemetry.update();

        }

    }
}
