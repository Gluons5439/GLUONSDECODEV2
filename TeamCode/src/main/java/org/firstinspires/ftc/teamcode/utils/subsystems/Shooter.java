package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class Shooter extends SubsystemBase {
    public Motor shooter1;
    public Motor shooter2;
    public ServoEx hood;
    public ServoEx stopper;

    public static double P = 0.006;
    public static double D = 0.0;
    public static double F = 0.0008;
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 60;

    public static double STOPPER_OPEN = 0.365;
    public static double STOPPER_CLOSED = 0.473;
    public static double HOOD_MIN = 0.267;
    public static double HOOD_MAX = 0.16;

    public static double HOOD_NEAR = 0.425;

    public static final double MAX_VELO = 1600;
    public static double idleVeloMultiplier = 0.0;

    public static double multiplier = 0.65;
    public static double VELO_NEAR = MAX_VELO * multiplier;

    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.BARE);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hMap, "hood");
        stopper = new ServoEx(hMap, "stopper");

        shooter1.setInverted(true);
        controller.setTolerance(TOLERANCE);
        controller.setSetPoint(0);
    }

    public void update() {
        double velocity = getVelocity();
        double power = controller.calculate(velocity);
        setPower(power);

        controller.setP(P);
        controller.setD(D);
        controller.setF(F);

        VELO_NEAR = MAX_VELO * multiplier;
    }
    public void setVelocity(double velocity) {
        controller.setSetPoint(velocity);
    }

    public double getVelocity(){
        return -shooter1.getCorrectedVelocity();
    }

    public void setPower(double power){
        shooter1.set(power);
        shooter2.set(power);
    }

    public void closeStopper(){
        stopper.set(STOPPER_CLOSED);
    }

    public void openStopper(){
        stopper.set(STOPPER_OPEN);
    }

    public void resetHood(){
        setHoodPercent(0);
    }

    public void raiseHood(){
        setHoodPercent(HOOD_NEAR);
    }

    public void setHoodPercent(double percent) {
        hood.set(HOOD_MIN + (HOOD_MAX - HOOD_MIN) * percent);
    }
}