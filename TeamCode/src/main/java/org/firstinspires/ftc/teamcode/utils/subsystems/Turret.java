package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.utils.Mosby;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Turret extends SubsystemBase {
    public Motor motor;
    public double ticksPerRadian = 306.532420395;
    public static double p = 0;//3
    public static double d = .00;//0.03
    public PIDFController controller = new PIDFController(p, 0, d, 0);
    public double tolerance = 1;

    public boolean enableAim = false;
    public double homePos = 0;
    public Turret(HardwareMap hMap) {
        motor = new Motor(hMap, "turretMotor", Motor.GoBILDA.RPM_223);
        motor.stopAndResetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(true);
        controller.setTolerance(tolerance);
        homePos = 0;
    }

    public double getAngle(){
        double currentPos = motor.getCurrentPosition();
        return (currentPos) / ticksPerRadian;
    }

    public void setAngle(double angle){
        controller.setSetPoint(wrapToPi(angle));
    }

    public void update() {

        if(enableAim){
            Pose pos = Mosby.drivetrain.follower.getPose();

            double deltaX = Mosby.goal.getX() - pos.getX();
            double deltaY = Mosby.goal.getY() - pos.getY();

            double targetAngle = Math.atan2(deltaY, deltaX);

            double robotAngle = Mosby.drivetrain.follower.getHeading();
            setAngle(targetAngle - robotAngle);
        }else {
            setAngle(homePos);

        }

        controller.setP(p);
        controller.setD(d);
        double angle = getAngle();
        Storage.turretAngle = angle;
        motor.set(controller.calculate(angle));
    }


    public static double wrapToPi(double radians) {
        double twoPi = 2 * Math.PI;
        double result = radians % twoPi;

        if (result <= -Math.PI) {
            result += twoPi;
        } else if (result > Math.PI) {
            result -= twoPi;
        }

        return result;
    }
}