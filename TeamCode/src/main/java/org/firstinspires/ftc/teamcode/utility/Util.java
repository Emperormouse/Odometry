package org.firstinspires.ftc.teamcode.utility;

public class Util {
    public static double normalizeRadian(double angle) {
        while (angle < 0)
            angle += (2 * Math.PI);
        while (angle >= 2 * Math.PI)
            angle -= (2 * Math.PI);

        return angle;
    }
}
