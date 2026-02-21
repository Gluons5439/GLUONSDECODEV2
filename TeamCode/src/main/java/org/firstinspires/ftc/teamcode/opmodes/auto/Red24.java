package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths2;
import org.firstinspires.ftc.teamcode.utils.Mosby;

@Autonomous(preselectTeleOp="TeleOp")
public class Red24 extends CommandOpMode {

    Paths2 paths2;

    @Override
    public void initialize() {
        // Initialize for the new Red Square position
        Mosby.init(hardwareMap, Mosby.MatchState.AUTO, Mosby.Alliance.REDSQ);
        paths2 = new Paths2(Mosby.drivetrain.follower, Mosby.Alliance.REDSQ);

        Mosby.drivetrain.follower.setMaxPower(1);

        schedule(new SequentialCommandGroup(
                // preloaded 3
                Mosby.reset(),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.ScorePre),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new WaitCommand(125),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                // intake middle
                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.collect1),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.Score1),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new WaitCommand(150),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                // gate 1
                // new SequentialCommandGroup(
                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(0.8)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.collect2,true),
                new WaitUntilCommand(() -> !Mosby.drivetrain.follower.isBusy()),
                // ),

                new WaitCommand(2000),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.score2),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new WaitCommand(125),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                // gate
                // new SequentialCommandGroup(
                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(0.8)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.collect3, true),
                new WaitUntilCommand(() -> !Mosby.drivetrain.follower.isBusy()),
                //),

                new WaitCommand(2200),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.score3),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new WaitCommand(125),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                // last row intake
                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(0.95)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.collect4),
                new WaitCommand(100),
                new InstantCommand(() -> Mosby.intake.setPower(0)),

                new FollowPathCommand(Mosby.drivetrain.follower, paths2.score4),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new WaitCommand(125),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.collect5),
                new WaitCommand(100),
                new InstantCommand(() -> Mosby.intake.setPower(0)),

                new FollowPathCommand(Mosby.drivetrain.follower, paths2.score5),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new WaitCommand(125),
                Mosby.shootWithIntake(),
                Mosby.reset(),
                new InstantCommand(() ->Mosby.shooter.autoPower(false,false)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths2.leave)

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