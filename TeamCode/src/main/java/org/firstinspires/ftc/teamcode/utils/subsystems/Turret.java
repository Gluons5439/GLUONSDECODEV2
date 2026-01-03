package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.utils.Snoopy;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Turret extends SubsystemBase {
    public Motor motor;
    public double ticksPerRadian = 216.2809573;
    public static double p = 3;
    public static double d = .03;
    public PIDFController controller = new PIDFController(p, 0, d, 0);
    public double tolerance = 1;

    public boolean enableAim = false;

    public Turret(HardwareMap hMap) {
        motor = new Motor(hMap, "turret", Motor.GoBILDA.BARE);
        motor.stopAndResetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(true);
        controller.setTolerance(tolerance);
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
            Pose pos = Snoopy.drivetrain.follower.getPose();

            double deltaX = Snoopy.goal.getX() - pos.getX();
            double deltaY = Snoopy.goal.getY() - pos.getY();

            double targetAngle = Math.atan2(deltaY, deltaX);

            double robotAngle = Snoopy.drivetrain.follower.getHeading();
            setAngle(targetAngle - robotAngle);
        }else{
            setAngle(0);
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