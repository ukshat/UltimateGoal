package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Register this Op Mode on the Android phone
@TeleOp(name = "Color Sensor Determination")

public class ColorDetermination extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    int red;
    int green;
    int blue;
    int alpha;

    String color;

    @Override
    public void runOpMode() {
        colorSensor = (RevColorSensorV3) hardwareMap.get("ColorSensorLeft");
        waitForStart();
        while(opModeIsActive()){
            updateColors();
            color = "gray";
            if (green > 430) color = "white";
            else if (blue > 430) color = "blue";
            else if (red > 430) color = "red";
            telemetry.addData(color, "R: " + red + "G:" + green + " B: " + blue + " A: " + alpha);
            telemetry.update();
        }
    }
    void updateColors(){
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        alpha = colorSensor.alpha();
    }
}