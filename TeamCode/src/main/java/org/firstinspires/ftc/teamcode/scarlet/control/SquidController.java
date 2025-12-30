package org.firstinspires.ftc.teamcode.scarlet.control;

public class SquidController implements Controller {
    public double kSq;
    private double target;
    private double tolerance;

    public SquidController(double kSq, double tolerance) {
        this.kSq = kSq;
        this.tolerance = tolerance;
    }

    public double calculate(double error) { return Math.sqrt(Math.abs(error * kSq)) * Math.signum(error); }

    public void setConstants(double... constants) {
        kSq = constants[0];
    }

    @Override
    public boolean atTarget() {
        return atTarget(target);
    }

    public boolean atTarget(double pos) { return Math.abs(target - pos) < tolerance; }

    public void setTarget(double target) {
        this.target = target;
    }

    @Override
    public void setTolerance(double tol) {
        tolerance = tol;
    }

    @Override
    public void reset() {

    }

    public double getTarget() {
        return target;
    }
}