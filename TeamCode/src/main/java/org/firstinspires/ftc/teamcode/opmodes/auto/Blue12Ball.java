package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.Mosby;

@Autonomous(preselectTeleOp="TeleOp")
public class Blue12Ball extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Mosby.init(hardwareMap, Mosby.MatchState.AUTO, Mosby.Alliance.BLUE);
        paths = new Paths(Mosby.drivetrain.follower, Mosby.Alliance.BLUE);
        Mosby.drivetrain.follower.setMaxPower(0.8);

        schedule(new SequentialCommandGroup(
                new FollowPathCommand(Mosby.drivetrain.follower, paths.startToScore),

                Mosby.prime(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.reset(),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(0.8);
                    Mosby.intake.setPower(0);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakeGPP1),
                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakeGPP2),
                new WaitCommand(400),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    //Mosby.intake.setPower(0.5);
                    //Mosby.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.scoreGPP),

                Mosby.prime(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.reset(),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(0.9);
                    Mosby.intake.setPower(0);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakePGP1),
                new WaitCommand(150),
                new InstantCommand(() -> {
                    Mosby.intake.setPower(1);
                    Mosby.drivetrain.follower.setMaxPower(0.8);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakePGP2),
                new WaitCommand(400),


                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    //Mosby.intake.setPower(0.5);
                   // Mosby.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.scorePGP),

                Mosby.prime(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.reset(),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(0.8);
                    Mosby.intake.setPower(0);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakePPG1),
                new WaitCommand(350),
                new InstantCommand(() -> {
                    Mosby.intake.setPower(1);
                    Mosby.drivetrain.follower.setMaxPower(0.6);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakePPG2),
                new WaitCommand(400),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(0.8);
                   // Mosby.intake.setPower(0.5);
                    //Mosby.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.scorePPG),

                Mosby.prime(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.shoot(),
                Mosby.reset(),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    Mosby.intake.setPower(0);
                    Mosby.intake.setMinPower(0);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.park)
        ));
    }

    @Override
    public void run() {
        super.run();
        Mosby.update();
        telemetry.addData("turret angle", Math.toDegrees(Mosby.turret.getAngle()));
        telemetry.addData("setpoint", Mosby.turret.controller.getSetPoint());
        telemetry.addData("goal", Mosby.goal);
        telemetry.update();
    }
}
