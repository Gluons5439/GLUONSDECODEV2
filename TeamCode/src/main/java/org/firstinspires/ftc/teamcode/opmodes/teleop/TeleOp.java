package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.TedMosby;
import org.firstinspires.ftc.teamcode.utils.Storage;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {

    public static double increment = 0.0875;

    @Override
    public void initialize() {
        TedMosby.init(hardwareMap, TedMosby.MatchState.TELEOP, Storage.alliance);

        Command prime = TedMosby.prime();
        Command shoot = TedMosby.shootOptimized();

        GamepadEx arvind = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        arvind.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                        }),
                        TedMosby.reset()
                ));

        arvind.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(prime);

        arvind.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(shoot);



        toolOp.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                        }),
                        TedMosby.reset()
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(prime);

        toolOp.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(shoot);

        toolOp.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            TedMosby.intake.setMinPower(-1);
                            TedMosby.intake.setPower(-1);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(() -> {
                            TedMosby.intake.setPower(1);
                            TedMosby.intake.setMinPower(1);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(() -> {
                            TedMosby.intake.setPower(0);
                            TedMosby.intake.setMinPower(0);
                        })
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new InstantCommand(() -> TedMosby.shooter.openStopper())
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            double pos = TedMosby.turret.controller.getSetPoint();
            TedMosby.turret.homePos = pos - increment;
        }));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            double pos = TedMosby.turret.controller.getSetPoint();
            TedMosby.turret.homePos = pos + increment;
        }));
        toolOp.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(new InstantCommand(() -> {
            TedMosby.init(hardwareMap, TedMosby.MatchState.TELEOP, Storage.alliance);
        }));
    }

    public void run() {
        super.run();
        TedMosby.update();
        TedMosby.drivetrain.drive(gamepad1);
        TedMosby.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("error", TedMosby.shooter.controller.getPositionError());
        telemetry.addData("atSetPoint", TedMosby.shooter.controller.atSetPoint());
        telemetry.addData("velo", TedMosby.shooter.getVelocity());
        telemetry.addData("storage angle", Storage.turretAngle);
        telemetry.update();
    }


}