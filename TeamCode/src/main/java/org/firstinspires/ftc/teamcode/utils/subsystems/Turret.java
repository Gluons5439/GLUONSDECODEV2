package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

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
    public static double MIN_ANGLE_DEGREES = -130;
    public static double MAX_ANGLE_DEGREES = 230;

    // The motor encoder is reset whenever this subsystem is rebuilt. This offset keeps
    // its zero aligned with the last physical turret angle saved in Storage.
    private double encoderAngleOffsetRadians;

    public Turret(HardwareMap hMap) {
        encoderAngleOffsetRadians = Storage.turretAngle;

        motor = new Motor(hMap, "turretMotor", Motor.GoBILDA.RPM_312);
        motor.stopAndResetEncoder();
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(false);
        controller.setTolerance(tolerance);
        homePos = 0;
    }

    public double getAngle(){
        double currentPos = motor.getCurrentPosition();
        // Add the saved angle to movement measured since the encoder reset.
        return encoderAngleOffsetRadians + currentPos / ticksPerRadian;
    }

    public void setAngle(double angle){
        double minAngle = Math.toRadians(MIN_ANGLE_DEGREES);
        double maxAngle = Math.toRadians(MAX_ANGLE_DEGREES);
        double wrappedAngle = wrapToPi(angle);

        // A wrapped target such as -170 degrees is physically the same as
        // +190 degrees. Use whichever equivalent lies inside the safe range.
        double[] equivalentAngles = {
                wrappedAngle,
                wrappedAngle + 2 * Math.PI,
                wrappedAngle - 2 * Math.PI
        };

        for (double equivalentAngle : equivalentAngles) {
            if (equivalentAngle >= minAngle && equivalentAngle <= maxAngle) {
                leAngle = equivalentAngle;
                controller.setSetPoint(leAngle);
                return;
            }
        }

        // The target is in the wire-protection deadzone. Stop at the nearest
        // safe boundary instead of commanding the turret through that zone.
        double distanceToMin = Math.abs(wrapToPi(wrappedAngle - minAngle));
        double distanceToMax = Math.abs(wrapToPi(wrappedAngle - maxAngle));
        leAngle = distanceToMin <= distanceToMax ? minAngle : maxAngle;

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
