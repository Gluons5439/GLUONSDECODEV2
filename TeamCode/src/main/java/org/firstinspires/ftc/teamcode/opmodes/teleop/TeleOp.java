package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Snoopy;
import org.firstinspires.ftc.teamcode.utils.Storage;
import org.firstinspires.ftc.teamcode.utils.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        Snoopy.init(hardwareMap, Snoopy.MatchState.TELEOP, Storage.alliance);

        Command prime = Snoopy.prime();
        Command shoot = Snoopy.shootOptimized();

        GamepadEx arvind = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        arvind.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                        }),
                        Snoopy.reset()
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
                        Snoopy.reset()
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(prime);

        toolOp.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(shoot);

        toolOp.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            Snoopy.intake.setMinPower(-1);
                            Snoopy.intake.setPower(-1);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(() -> {
                            Snoopy.intake.setPower(1);
                            Snoopy.intake.setMinPower(1);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(() -> {
                            Snoopy.intake.setPower(0);
                            Snoopy.intake.setMinPower(0);
                        })
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new InstantCommand(() -> Snoopy.shooter.openStopper())
                );
    }

    public void run() {
        super.run();
        Snoopy.update();
        Snoopy.drivetrain.drive(gamepad1);
        Snoopy.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("error", Snoopy.shooter.controller.getPositionError());
        telemetry.addData("atSetPoint", Snoopy.shooter.controller.atSetPoint());
        telemetry.addData("velo", Snoopy.shooter.getVelocity());
        telemetry.update();
    }


}