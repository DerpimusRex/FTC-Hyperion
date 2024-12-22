package org.firstinspires.ftc.teamcode.FTC_Into_The_Deep.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    double lastError;
    double integralSum;

    double p, d, i;
    ElapsedTime timer = new ElapsedTime();
    public PIDController(double Kp, double Ki, double Kd) {
        p = Kp;
        d = Kd;
        i = Ki;
    }

    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - lastError)/timer.seconds();
        integralSum += error * timer.seconds();

        lastError = error;

        timer.reset();

        return error*p + derivative*d + integralSum*i;
    }
}
