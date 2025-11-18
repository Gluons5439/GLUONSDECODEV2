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

    public static double p = 0.001;
    public static double d = 0;
    public static double f = 0.0008;
    public PIDFController controller = new PIDFController(p, 0, d, f);
    public static double tolerance = 5; // in ticks

    // TODO find these values
    public static double stopperOpen = 0.3;
    public static double stopperClosed = 0.45;
    public static double hoodMin = 0.267;
    public static double hoodMax = 0.16;

    public static double hoodShootNear = hoodMax;

    public static double maxVelo = 1600;

    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.BARE);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hMap, "hood");
        stopper = new ServoEx(hMap, "stopper");

        shooter1.setInverted(true);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        stopper.set(stopperClosed);
        hood.set(hoodMin);
    }

    public void update() {
        double power = controller.calculate(-shooter1.getCorrectedVelocity());
        shooter1.set(power);
        shooter2.set(power);
    }
    public void setTargetVelo(double velo) {
        controller.setSetPoint(velo);
    }
    public void setHood(double pos) {
        hood.set(pos);
    }
    public void setStopper(double pos) {
        stopper.set(pos);
    }
}
