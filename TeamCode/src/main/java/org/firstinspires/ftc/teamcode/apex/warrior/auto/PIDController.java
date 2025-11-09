package org.firstinspires.ftc.teamcode.apex.warrior.auto;

public class PIDController {
    double kP, kI, kD;
    double integral = 0, lastError = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double error, double dt) {
        integral = integral + error * dt;
        if (integral >= 1)
            integral = 1;
        if (integral <= -1)
            integral = -1;

        double derivative = (error - lastError) / dt;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }
}
