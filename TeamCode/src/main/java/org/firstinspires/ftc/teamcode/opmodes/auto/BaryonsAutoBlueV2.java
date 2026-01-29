package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.Mosby;

@Autonomous(preselectTeleOp="TeleOp")
public class BaryonsAutoBlueV2 extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Mosby.init(hardwareMap, Mosby.MatchState.AUTO, Mosby.Alliance.BLUE);
        paths = new Paths(Mosby.drivetrain.follower, Mosby.Alliance.BLUE);
        Mosby.drivetrain.follower.setMaxPower(1);

        schedule(new SequentialCommandGroup(
                new FollowPathCommand(Mosby.drivetrain.follower, paths.startToScore1),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    Mosby.intake.setPower(1);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1),
                new WaitCommand(200),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.checkIntakepgp1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1again),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    Mosby.intake.setPower(0);
                    Mosby.drivetrain.follower.setMaxPower(1);
                }),

                new FollowPathCommand(Mosby.drivetrain.follower, paths.shootpgp),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    Mosby.intake.setPower(1);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1),
                new WaitCommand(200),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.checkIntakepgp1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1again),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    Mosby.intake.setPower(0);
                    Mosby.drivetrain.follower.setMaxPower(1);
                }),

                new FollowPathCommand(Mosby.drivetrain.follower, paths.shootpgp),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1),
                new WaitCommand(200),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.checkIntakepgp1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1again),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    Mosby.intake.setPower(0);
                    Mosby.drivetrain.follower.setMaxPower(1);
                }),

                new FollowPathCommand(Mosby.drivetrain.follower, paths.shootpgp),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),
/**
 new InstantCommand(() -> Mosby.intake.setPower(1)),
 new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1),
 new WaitCommand(200),
 new FollowPathCommand(Mosby.drivetrain.follower, paths.checkIntakepgp1),
 new FollowPathCommand(Mosby.drivetrain.follower, paths.intakepgp1again),
 new WaitCommand(200),
 new InstantCommand(() -> {
 Mosby.intake.setPower(0);
 Mosby.drivetrain.follower.setMaxPower(1);
 }),

 new FollowPathCommand(Mosby.drivetrain.follower, paths.shootpgp),
 Mosby.prime(),
 new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
 Mosby.shootWithIntake(),
 Mosby.reset(),
 **/
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
        telemetry.addData("shooter current velocity", Mosby.shooter.getVelocity());
        telemetry.addData("shooter set Velocity", Mosby.shooter.power);
        telemetry.update();
    }
}
