package org.firstinspires.ftc.teamcode.apex.warrior.auto;

public class MecanumKinematics {
    public static double[] calculateWheelPowers(double vx, double vy, double omega) {
        double fl = vx - vy - omega;
        double fr = vx - vy + omega;
        double bl = vx + vy - omega;
        double br = vx + vy + omega;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 0.2) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        return new double[]{fl, fr, bl, br};
    }
}
