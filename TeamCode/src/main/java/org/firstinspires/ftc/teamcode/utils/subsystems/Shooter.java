package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    public PIDFController controller = new PIDFController(p, 0, d, 0);
    public double tolerance = 25; // in ticks

    // TODO find these values
    public static double stopperOpen = 0;
    public static double stopperClosed = 1;
    public static double hoodFar = 0.2;
    public static double hoodClose = 0.7;
    public static double hoodMid = 0.5;

    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.BARE);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hMap, "hood");
        stopper = new ServoEx(hMap, "stopper");

        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
    }

    public void update() {
        double power = controller.calculate(shooter1.getCorrectedVelocity());
    }
}
