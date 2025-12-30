package org.firstinspires.ftc.teamcode.scarlet;

public interface Controller {
    double calculate(double pos);
    double calculate(double pos, double target);

    boolean atTarget();
    boolean atTarget(double target);
    double getTarget();
    void setTarget(double target);

    void setTolerance(double tol);

    void reset();
}
