package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

@TeleOp(name = "Shooter PIDF Tuning", group = "Test")
@Configurable
public class ShooterPIDFTuning extends OpMode {

    // ======= TUNABLES (LIVE VIA DASHBOARD / PANELS) =======
    public static double P = 0.001;
    public static double F = 0.000385;

    public static double TARGET_VELOCITY = 1600; // ticks/sec
    public static boolean SHOOTER_ON = false;
    public static double INTAKE_POWER = 0;

    // =====================================================

    private Motor shooter1;
    private Motor shooter2;

    private Motor intake;

    private PIDFController controller;

    @Override
    public void init() {
        shooter1 = new Motor(hardwareMap, "shooterMotor", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hardwareMap, "shooterMotor2", Motor.GoBILDA.BARE);
        intake = new Motor(hardwareMap, "Intake", Motor.GoBILDA.RPM_1150);

       shooter1.setInverted(true);

        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        //shooter1.setInverted(true);
        //shooter2.setInverted(true);

        controller = new PIDFController(P, 0, 0, F);
        controller.setTolerance(50);
        controller.setSetPoint(0);
    }

    @Override
    public void loop() {

        // Update gains live
        controller.setP(P);
        controller.setF(F);

        double currentVelocity = getVelocity();
        double power = 0;

        if (SHOOTER_ON) {
            controller.setSetPoint(TARGET_VELOCITY);

            power = controller.calculate(currentVelocity);

            // Shooter should never reverse
            power = clamp(power, 0.0, 1.0);
        } else {
            controller.reset();
            power = 0;
        }
        intake.set(INTAKE_POWER);

        setPower(power);

        // ===== TELEMETRY =====
        telemetry.addData("Shooter On", SHOOTER_ON);
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", TARGET_VELOCITY - currentVelocity);
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Power Output", power);
        telemetry.update();
    }

    private double getVelocity() {
        return -shooter2.getCorrectedVelocity();
    }

    private void setPower(double power) {
        shooter1.set(-power);
        shooter2.set(power);
    }
}
