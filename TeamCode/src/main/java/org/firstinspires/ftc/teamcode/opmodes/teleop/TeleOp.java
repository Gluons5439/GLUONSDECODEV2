package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
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

        Command prime = Mosby.prime();
        Command shoot = Mosby.shootOptimized();

        GamepadEx arvind = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        arvind.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                        }),
                        Mosby.reset()
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
                        Mosby.reset()
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(prime);

        toolOp.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(shoot);

        toolOp.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            Mosby.intake.setMinPower(-1);
                            Mosby.intake.setPower(-1);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(() -> {
                            Mosby.intake.setPower(1);
                            Mosby.intake.setMinPower(1);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(() -> {
                            Mosby.intake.setPower(0);
                            Mosby.intake.setMinPower(0);
                        })
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new InstantCommand(() -> Mosby.shooter.openStopper())
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            double pos = Mosby.turret.controller.getSetPoint();
            Mosby.turret.homePos = pos - increment;
        }));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            double pos = Mosby.turret.controller.getSetPoint();
            Mosby.turret.homePos = pos + increment;
        }));
        toolOp.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(new InstantCommand(() -> {
            Mosby.init(hardwareMap, Mosby.MatchState.TELEOP, Storage.alliance);
        }));
    }

    public void run() {
        super.run();
        Mosby.update();
        Mosby.drivetrain.drive(gamepad1);
        Mosby.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("error", Mosby.shooter.controller.getPositionError());
        telemetry.addData("atSetPoint", Mosby.shooter.controller.atSetPoint());
        telemetry.addData("velo", Mosby.shooter.getVelocity());
        telemetry.addData("storage angle", Storage.turretAngle);
        telemetry.update();
    }


}