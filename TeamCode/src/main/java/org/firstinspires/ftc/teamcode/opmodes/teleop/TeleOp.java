package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Snoopy;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        Snoopy.init(hardwareMap, Snoopy.MatchState.TELEOP, Snoopy.Alliance.BLUE);

        Command prime = Snoopy.prime();
        Command shoot = Snoopy.shootOptimized();

        GamepadEx driverOp = new GamepadEx(gamepad1);

        driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            prime.cancel();
                            shoot.cancel();
                        }),
                        Snoopy.reset()
                ));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(prime);

        driverOp.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(shoot);

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