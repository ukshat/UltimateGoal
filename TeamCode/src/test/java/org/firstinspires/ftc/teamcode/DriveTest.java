package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

public class DriveTest {

    @Test
    public void leftUp() {
        double[] powers = SimpleDriveCode.calculateMotorPower(-0.5, 0.5, 0);
        assertArrayEquals(powers, new double[]{0, 1, 1, 0}, 0.001);
    }

    @Test
    public void rightUp() {
        double[] powers = SimpleDriveCode.calculateMotorPower(0.5, 0.5, 0);
        assertArrayEquals(powers, new double[]{1, 0, 0, 1}, 0.001);
    }

    @Test
    public void back() {
        double[] powers = SimpleDriveCode.calculateMotorPower(0, -1, 0);
        assertArrayEquals(powers, new double[]{-1, -1, -1, -1}, 0.001);
    }

    @Test
    public void left() {
        double[] powers = SimpleDriveCode.calculateMotorPower(-1, 0, 0);
        assertArrayEquals(powers, new double[]{-1, 1, 1, -1}, 0.001);
    }

    @Test
    public void leftDown() {
        double[] powers = SimpleDriveCode.calculateMotorPower(-0.5, -0.5, 0);
        assertArrayEquals(powers, new double[]{-1, 0, 0, -1}, 0.001);
    }


}