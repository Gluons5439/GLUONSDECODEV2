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

    public static double P = 0.000362;//0.006
    public static double D = 0.0;
    public static double F = 0.000005;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 40;

    public static double STOPPER_OPEN = 0.05;
    public static double STOPPER_CLOSED = 0.267;
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
    public boolean shooterBlah;


    public Shooter(HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooterMotor", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooterMotor2", Motor.GoBILDA.BARE);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hMap, "HoodServo");
        stopper = new ServoEx(hMap, "StopperServo");
        transfer = new ServoEx(hMap, "TransferServo");

        shooter1.setInverted(true);
        controller.setTolerance(TOLERANCE);
        controller.setSetPoint(0);
        lutVelocity.add(31.25, 1600);
        lutVelocity.add(45.25, 1720);
        lutVelocity.add(51, 1840);
        lutVelocity.add(59.5, 1860);
        lutVelocity.add(65.75, 1960);
        lutVelocity.add(78.25, 1960);
        lutVelocity.add(88.25, 2060);
        lutVelocity.add(99.25, 2200);
        lutVelocity.add(115, 2340);
        lutVelocity.add(131.25, 2380);
        lutVelocity.add(138.5, 2430);
        lutVelocity.add(148,2500);

        lutHood.add(31.25, 0.1);
        lutHood.add(45.25, 0.3);
        lutHood.add(51, 0.33);
        lutHood.add(59.5, 0.36);
        lutHood.add(65.75, 0.38);
        lutHood.add(78.25, 0.48);
        lutHood.add(88.25, 0.48);
        lutHood.add(99.25, 0.54);
        lutHood.add(115, 0.54);
        lutHood.add(131.25, 0.62);
        lutHood.add(138.5, 0.62);
        lutHood.add(148,0.62);
        lutVelocity.createLUT();
        lutHood.createLUT();
        //lutHood.add();
       // Pose pos = Mosby.drivetrain.follower.getPose();

        shooterBlah = false;


    }

    public void update() {


       // controller.setP(P);
        //controller.setD(D);
        //controller.setF(F);


       //this.autoPower(shooterBlah);
        distance = Math.hypot(Mosby.goal.getX() - (Mosby.drivetrain.follower.getPose()).getX(), Mosby.goal.getY() - (Mosby.drivetrain.follower.getPose()).getY());
        double velocity = getVelocity();
        double power = controller.calculate(velocity);
        setPower(power);
    }
    public void setShooter(boolean s) {
        shooterBlah = s;
    }

    public void setVelocity(double velocity) {
        controller.setSetPoint(velocity);
    }

    public double getVelocity() {
        return -shooter1.getCorrectedVelocity();
    }

    public void setPower(double power) {
        shooter1.set(-power);
        shooter2.set(power);
    }
    public void autoPower(boolean shooterOn , boolean hoodOn) {
        if (shooterOn){
            //  double velocity = clamp(lutVelocity.get(distance), 0, 1);
          //  double hoodVal = clamp(lutHood.get(distance), 0, 0.7);
            double velocity = lutVelocity.get(distance);
            double power = controller.calculate(velocity);
        shooter1.set(-power);
        shooter2.set(power);
    }
        if(hoodOn)
        {
            double hoodVal = lutHood.get(distance);
            hood.set(hoodVal);
        }
        else
        {
            shooter1.set(0);
            shooter2.set(0);
            hood.set(0);

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
}
