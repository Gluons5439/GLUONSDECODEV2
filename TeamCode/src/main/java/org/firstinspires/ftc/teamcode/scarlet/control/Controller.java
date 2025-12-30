package org.firstinspires.ftc.teamcode.scarlet.control;

public interface Controller {
    double calculate(double error);
    void setConstants(double... constants);

    boolean atTarget();
    boolean atTarget(double target);
    double getTarget();
    void setTarget(double target);

    void setTolerance(double tol);

    void reset();
}
