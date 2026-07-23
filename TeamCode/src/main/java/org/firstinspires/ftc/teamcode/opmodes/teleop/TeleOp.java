package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Mosby;
import org.firstinspires.ftc.teamcode.utils.Storage;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {

    public static double increment = 0.0875;

    @Override
    public void initialize() {
        Mosby.init(hardwareMap, Mosby.MatchState.TELEOP, Storage.alliance);
        Mosby.update();

        Command prime = Mosby.prime();
        Command shoot = Mosby.shoot();
        Command shootWithIntake = Mosby.shootWithIntake();


        GamepadEx rishi = new GamepadEx(gamepad1);
        GamepadEx aaryan = new GamepadEx(gamepad2);

        // Aaryan controls
        rishi.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(prime);

        rishi.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shoot);

        rishi.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(shootWithIntake);
        rishi.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    Mosby.reset();
                }) );
        rishi.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    if(Storage.alliance == Mosby.Alliance.BLUE || Storage.alliance == Mosby.Alliance.BLUESQ) {
                        Pose b = new Pose(135.5, 7.8125, Math.toRadians(90));
                        Mosby.drivetrain.follower.setPose(b);
                        Storage.pose = b;

                    } else {
                        Pose r = new Pose(8.5, 7.8125, Math.toRadians(90));
                        Mosby.drivetrain.follower.setPose(r);
                        Storage.pose = r;
                    }
                }) );




        aaryan.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            double pos = Mosby.turret.controller.getSetPoint();
            Mosby.turret.homePos = pos - increment;
        }));

        aaryan.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            double pos = Mosby.turret.controller.getSetPoint();
            Mosby.turret.homePos = pos + increment;
        }));
        // Redundant shooter stop: persist a zero-RPM idle target and cut power immediately.
        rishi.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            Mosby.shooter.runIdle(0);
            Mosby.shooter.setPower(0);
        }));

        // Business presets lock the turret at home and hold fixed shooter RPM/hood values.
        rishi.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            Mosby.turret.enableAim = false;
            Mosby.turret.AUTOenableAim = false;
            Mosby.shooter.runBusinessMode(3300, 0.22);
        }));

        rishi.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            Mosby.turret.enableAim = false;
            Mosby.turret.AUTOenableAim = false;
            Mosby.shooter.runBusinessMode(4000, 0.30);
        }));

        aaryan.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(new InstantCommand(() -> {
            Mosby.init(hardwareMap, Mosby.MatchState.TELEOP, Storage.alliance);
        }));
    }

    public void run() {
        super.run();
        Mosby.update();
        Mosby.drivetrain.drive(gamepad1);
        double gp1Intake = gamepad1.right_trigger - gamepad1.left_trigger;
        double gp2Intake = gamepad2.right_trigger - gamepad2.left_trigger;

// Take whichever input has greater magnitude
        double intakePower = Math.abs(gp1Intake) > Math.abs(gp2Intake)
                ? gp1Intake
                : gp2Intake;

        Mosby.intake.setPower(intakePower);



        telemetry.addData("error", Mosby.shooter.controller.getPositionError());
        telemetry.addData("position", Mosby.drivetrain.follower.getPose().getX());
        telemetry.addData("position", Mosby.drivetrain.follower.getPose().getY());
        telemetry.addData("heading", Mosby.drivetrain.follower.getPose().getHeading());
        telemetry.addData("goal", Mosby.goalShooter.getX());
        telemetry.addData("goal", Mosby.goalShooter.getY());
        telemetry.addData("distance", Mosby.shooter.distance);
        telemetry.addData("ActualVelo", Mosby.shooter.controller.getSetPoint());
        telemetry.addData("atSetPoint", Mosby.shooter.controller.atSetPoint());
        telemetry.addData("velo", Mosby.shooter.getVelocity());
        telemetry.addData("storage angle", Storage.turretAngle);
        telemetry.update();
    }


}