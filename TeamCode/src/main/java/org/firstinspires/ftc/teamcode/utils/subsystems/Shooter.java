
package org.firstinspires.ftc.teamcode.utils.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.utils.Mosby;
import org.firstinspires.ftc.teamcode.utils.Storage;


@Configurable
public class Shooter extends SubsystemBase {
    public Motor shooter1;
    public Motor shooter2;
    public ServoEx hood;
    public ServoEx stopper;
    public ServoEx transfer;

    public static double P = 0.001;//0.006 0.000389
    public static double D = 0.0;
    public static double F =0.00038;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 80;

    public static double STOPPER_OPEN = 0.1;
    public static double STOPPER_CLOSED = 0.3;
    public static double TRANSFER_UP = 0.85;
    public static double TRANSFER_DOWN = 0.5;
    public static double HOOD_MIN = 0;
    public static double HOOD_MAX = 1;

    public static double HOOD_NEAR = 0;


    public static double idleVeloMultiplier = 0.0;

    public static double multiplier = 0.65;

    InterpLUT lutVelocity = new InterpLUT();
    InterpLUT lutHood = new InterpLUT();
    public double distance;
    public double power;
    public boolean shooterBlah;
    public Pose pos;
    //double currentVelocity = 0;

    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooterMotor", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooterMotor2", Motor.GoBILDA.BARE);
        //shooter1.setInverted(true);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);


        hood = new ServoEx(hMap, "HoodServo");
        stopper = new ServoEx(hMap, "StopperServo");
        transfer = new ServoEx(hMap, "TransferServo");
        transfer.set(TRANSFER_DOWN);

        shooter1.setInverted(true);
        controller.setTolerance(TOLERANCE);
        controller.setSetPoint(0);
        lutVelocity.add(0, 1490);
        lutVelocity.add(27.5, 1500);
        lutVelocity.add(39, 1560);
        lutVelocity.add(52.5, 1620);
        lutVelocity.add(65, 1660);
        lutVelocity.add(75.5, 1720);
        lutVelocity.add(88.5, 1780);
        lutVelocity.add(105.5, 1880);
        lutVelocity.add(120.5, 1940);
        lutVelocity.add(133.5, 2060);
        lutVelocity.add(148.5, 2220);
        lutVelocity.add(163.5, 2280);

        lutHood.add(0, 0);
        lutHood.add(27.5, 0.02);
        lutHood.add(39, 0.134);
        lutHood.add(52.5, 0.16);
        lutHood.add(65, 0.35);
        lutHood.add(75.5, 0.35);
        lutHood.add(88.5, 0.39);
        lutHood.add(105.5, 0.42);
        lutHood.add(120.5, 0.44);
        lutHood.add(133.5, 0.5);
        lutHood.add(148.5, 0.52);
        lutHood.add(163.5, 0.56);
        lutVelocity.createLUT();
        lutHood.createLUT();
        pos = Mosby.drivetrain.follower.getPose();
        controller.setP(P);
        controller.setF(F);


        shooterBlah = false;
        distance = Math.hypot(Mosby.goalShooter.getX()-Storage.pose.getX(),Mosby.goalShooter.getY()-Storage.pose.getY());


    }

    public void update() {
        if (!shooterBlah) {
            controller.setSetPoint(0);
            setPower(0);
            return;
        }
        pos = Mosby.drivetrain.follower.getPose();

        distance = Math.hypot(
                Mosby.goalShooter.getX() - pos.getX(),
                Mosby.goalShooter.getY() - pos.getY()
        );

        double currentVelocity = getVelocity();
        double targetVelocity = lutVelocity.get(distance);

        controller.setSetPoint(targetVelocity);

        power = controller.calculate(currentVelocity);
        setPower(power);
    }


    public void setShooter(boolean s) {
        shooterBlah = s;
    }

    public void setVelocity(double velocity) {
        controller.setSetPoint(velocity);
        //currentVelocity = velocity;
    }

    public double getVelocity() {
        return -shooter1.getCorrectedVelocity();
    }

    public void setPower(double power) {
        power = clamp(power, -1.0, 1.0);
        shooter1.set(-power);
        shooter2.set(power);
    }

    public void autoPower(boolean shooterOn, boolean hoodOn) {
        shooterBlah = shooterOn;

        if (shooterOn) {
            controller.setP(P);
            controller.setF(F);
            controller.setSetPoint(lutVelocity.get(distance) );
        } else {
            controller.setSetPoint(0);
        }

        if (hoodOn) {
            hood.set(lutHood.get(distance));
        }
    }


    public void closeStopper() {
        stopper.set(STOPPER_CLOSED);
    }

    public void openStopper() {
        stopper.set(STOPPER_OPEN);
    }
    public void hitTransfer(){
        transfer.set(TRANSFER_UP);
    }
    public void downTransfer(){
        transfer.set(TRANSFER_DOWN);
    }

    public void resetHood() {
        setHoodPercent(0);
    }

    public void raiseHood() {
        setHoodPercent(HOOD_NEAR);
    }

    public void setHoodPercent(double percent) {
        hood.set( HOOD_MAX * percent);
    }
    public void setCurrentHoodPercent(double percent) {hood.set(hood.getRawPosition()*percent);}
}
