package org.firstinspires.ftc.teamcode;
import java.lang.Math.*;


/*
This is a math utility class. it contains constants and utility functions
Release 1.0
* */

public class MathUtils {
    /*
    This method has the math for ticks per rotation
     */
    public static int getTicksForOneRotation(){
        /*
            It takes 28 ticks for one rotation of the inner motor
            It then take 40 inner motor rotations for the shaft
            to rotate once
        * */
        return(28 * 40);
    }

    public static double getTicksForRadians(float radians){


        /*
        2pi radians = T Ticks
        Therefore, 1 radians = T/2pi ticks

        Meaning, x radians = X * T/2pi ticks
        * */

        double ticks = radians * MathUtils.getTicksForOneRotation()/2*Math.PI;

        return ticks;
    }
}
