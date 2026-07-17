package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.utils.Mosby;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Turret extends SubsystemBase {
    public Motor motor;
    public double ticksPerRadian = 222.57818791402;
    public static double p = 3;//3
    public static double d = .03;//0.03
    public PIDFController controller = new PIDFController(p, 0, d, 0);
    public double tolerance = 1;

    public boolean enableAim = false;
    public boolean AUTOenableAim = false;
    public double homePos = 0;
    public double leAngle =0;
    private static final double MIN_ANGLE = Math.toRadians(-130);
    private static final double MAX_ANGLE = Math.toRadians(175);
    public Turret(HardwareMap hMap) {
        motor = new Motor(hMap, "turretMotor", Motor.GoBILDA.RPM_312);
        motor.stopAndResetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(false);
        controller.setTolerance(tolerance);
        homePos = 0;
    }

    public double getAngle(){
        double currentPos = motor.getCurrentPosition();
        return wrapToPi((currentPos) / ticksPerRadian);
    }

    public void setAngle(double angle){
        leAngle = MathUtils.clamp(wrapToPi(angle),MIN_ANGLE, MAX_ANGLE);

        controller.setSetPoint(leAngle);
    }

    public void update() {

        if(enableAim || AUTOenableAim){
           setAngle(calculateTargetAngle(Mosby.goalShooter));
        } else {
            setAngle(homePos);
        }

        controller.setP(p);
        controller.setD(d);
        double angle = getAngle();
        Storage.turretAngle = angle;
        motor.set(controller.calculate(angle));



    }

    private double calculateTargetAngle(Vector2d goal)
    {
        Pose pos = Mosby.drivetrain.follower.getPose();

        double deltaX = Mosby.goal.getX() - pos.getX();
        double deltaY = Mosby.goal.getY() - pos.getY();

        double targetAngle = Math.atan2(deltaY, deltaX);

        double robotAngle = Mosby.drivetrain.follower.getHeading();
        return wrapToPi(targetAngle-robotAngle + homePos); // homePos for Buisness but targetAngle-robotAngle + homePos For actual
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