package org.firstinspires.ftc.teamcode.scarlet;

public class PIDFController implements Controller {
    public double p;
    public double i;
    public double d;
    public double f;

    private double tolerance;
    private double target;

    private double lastError = 0;
    private double lastTime = 0;
    private double integral = 0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.p = kP;
        this.i = kI;
        this.d = kD;
        this.f = kF;
    }

    public double calculate(double pos, double target) {
        setTarget(target);
        return calculate(pos);
    }

    public double calculate(double pos) {
        double error = target - pos;
        double currentTime = System.currentTimeMillis();

        if (lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return (p * error) + (f * target);
        }

        double deltaTime = (currentTime - lastTime) / 1000.0;

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        lastError = error;
        lastTime = currentTime;

        return (p * error)
                + (i * integral)
                + (d * derivative)
                + (f * target);
    }

    @Override
    public boolean atTarget() {
        return atTarget(target);
    }

    public boolean atTarget(double pos) {
        return Math.abs(target - pos) < tolerance;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setTolerance(double tol) {
        this.tolerance = tol;
    }

    public double getTarget() {
        return target;
    }

    public void reset() {
        lastError = 0;
        lastTime = 0;
        integral = 0;
    }
}
