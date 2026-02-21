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
public class tangentred extends CommandOpMode {
    Paths2 paths;

    @Override
    public void initialize() {
        Mosby.init(hardwareMap, Mosby.MatchState.AUTO, Mosby.Alliance.RED);
        paths = new Paths2(Mosby.drivetrain.follower, Mosby.Alliance.RED);
        Mosby.drivetrain.follower.setMaxPower(0.9);

        schedule(new SequentialCommandGroup(
                new FollowPathCommand(Mosby.drivetrain.follower, paths.ScorePre),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CollectLastRow),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.ScoreLastRow),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CollectHP1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_HP2),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP2_to_HP1),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_Score),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CollectHP1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_HP2),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP2_to_HP1),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_Score),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CollectHP1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_HP2),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP2_to_HP1),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_Score),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CollectOffHP),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.OffHP1_to_OffHP2),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.OffHP2_to_OffHP1),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.HP1_to_Score), // Back to shoot2
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset()
        ));
    }

    @Override
    public void run() {
        super.run();
        Mosby.update();
    }
}
