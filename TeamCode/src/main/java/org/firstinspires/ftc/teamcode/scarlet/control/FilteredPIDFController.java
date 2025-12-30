package org.firstinspires.ftc.teamcode.scarlet.control;

public class FilteredPIDFController implements Controller {
    public double p;
    public double i;
    public double d;
    public double f;
    public double a;

    private double tolerance;
    private double target;

    private double lastError = 0;
    private double lastTime = 0;
    private double integral = 0;
    private double previousDerivative = 0;

    public FilteredPIDFController(double kP, double kI, double kD, double kF, double a) {
        this.p = kP;
        this.i = kI;
        this.d = kD;
        this.f = kF;
        this.a = a;
    }

    @Override
    public double calculate(double error) {
        double currentTime = System.currentTimeMillis();


        if (lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return (p * error) + (f * target);
        }

        double deltaTime = (currentTime - lastTime) / 1000.0;
        double currentDerivative = 0;
        if (deltaTime > 0) {
            currentDerivative = (error - lastError) / deltaTime;
        }
        integral += error * deltaTime;

        double derivative = a * previousDerivative + (1- a) * currentDerivative;
        previousDerivative = currentDerivative;


        lastError = error;
        lastTime = currentTime;

        return (p * error)
                + (i * integral)
                + (d * derivative)
                + (f * target);
    }

    public void setConstants(double... constants) {
        p = constants[0];
        i = constants[1];
        d = constants[2];
        f = constants[3];
        a = constants[4];
    }

    @Override
    public boolean atTarget() {
        return atTarget(target);
    }

    @Override
    public boolean atTarget(double target) {
        return Math.abs(target - this.target) < tolerance;
    }

    @Override
    public double getTarget() {
        return target;
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
    }

    @Override
    public void setTolerance(double tol) {
        tolerance = tol;
    }

    @Override
    public void reset() {
        previousDerivative = 0;
        lastError = 0;
        lastTime = 0;
        integral = 0;
    }
}
