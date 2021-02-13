package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous(name = "Print Potentiometer Value")
public class Potentiometer_Test extends LinearOpMode {
    AnalogInput pot;

    @Override
    public void runOpMode() throws InterruptedException {
        pot = hardwareMap.analogInput.get("wobblepot");
        waitForStart();

        while(opModeIsActive()){
            sleep(20);
            telemetry.addData("Potentiometer", pot.getVoltage());
            telemetry.update();

        }

    }
}
