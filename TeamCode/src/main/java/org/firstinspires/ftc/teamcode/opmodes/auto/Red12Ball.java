package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.Snoopy;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Autonomous(preselectTeleOp="TeleOp")
public class Red12Ball extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Snoopy.init(hardwareMap, Snoopy.MatchState.AUTO, Snoopy.Alliance.RED);
        paths = new Paths(Snoopy.drivetrain.follower, Snoopy.Alliance.RED);
        Snoopy.drivetrain.follower.setMaxPower(0.8);

        schedule(new SequentialCommandGroup(

                new InstantCommand(() -> {
                    Snoopy.intake.setPower(0.5);
                    Snoopy.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.startToScore),

                Snoopy.shootOptimized(),

                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(0.8);
                    Snoopy.intake.setPower(0);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakeGPP1),
                new InstantCommand(() -> Snoopy.intake.setPower(1)),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakeGPP2),
                new WaitCommand(400),

                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(1);
                    Snoopy.intake.setPower(0.5);
                    Snoopy.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.scoreGPP),

                Snoopy.shootOptimized(),

                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(0.9);
                    Snoopy.intake.setPower(0);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakePGP1),
                new WaitCommand(150),
                new InstantCommand(() -> {
                    Snoopy.intake.setPower(1);
                    Snoopy.drivetrain.follower.setMaxPower(0.8);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakePGP2),
                new WaitCommand(400),


                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(1);
                    Snoopy.intake.setPower(0.5);
                    Snoopy.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.scorePGP),

                Snoopy.shootOptimized(),

                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(0.8);
                    Snoopy.intake.setPower(0);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakePPG1),
                new WaitCommand(350),
                new InstantCommand(() -> {
                    Snoopy.intake.setPower(1);
                    Snoopy.drivetrain.follower.setMaxPower(0.6);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakePPG2),
                new WaitCommand(400),

                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(0.8);
                    Snoopy.intake.setPower(0.5);
                    Snoopy.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.scorePPG),

                Snoopy.shootOptimized(),

                new InstantCommand(() -> {
                    Snoopy.drivetrain.follower.setMaxPower(1);
                    Snoopy.intake.setPower(0);
                    Snoopy.intake.setMinPower(0);
                }),
                new FollowPathCommand(Snoopy.drivetrain.follower, paths.park)
        ));
    }

    @Override
    public void run() {
        super.run();
        Snoopy.update();
        telemetry.addData("turret angle", Math.toDegrees(Snoopy.turret.getAngle()));
        telemetry.addData("setpoint", Snoopy.turret.controller.getSetPoint());
        telemetry.addData("goal", Snoopy.goal);
        telemetry.addData("storage angle", Storage.turretAngle);
        telemetry.update();
    }
}
