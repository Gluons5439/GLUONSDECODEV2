package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.TedMosby;

@Autonomous(preselectTeleOp="TeleOp")
public class Blue9Ball extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        TedMosby.init(hardwareMap, TedMosby.MatchState.AUTO, TedMosby.Alliance.BLUE);
        paths = new Paths(TedMosby.drivetrain.follower, TedMosby.Alliance.BLUE);
        TedMosby.drivetrain.follower.setMaxPower(0.7);

        schedule(new SequentialCommandGroup(

            //
            new InstantCommand(() -> {
                TedMosby.intake.setPower(0.5);
                TedMosby.intake.setMinPower(0.5);
            }),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.startToScore),

            TedMosby.shootOptimized(),

            new InstantCommand(() -> TedMosby.drivetrain.follower.setMaxPower(0.5)),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.intakeGPP1),
            new InstantCommand(() -> TedMosby.intake.setPower(1)),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.intakeGPP2),
            new WaitCommand(500),

            new InstantCommand(() -> {
                TedMosby.drivetrain.follower.setMaxPower(0.6);
                TedMosby.intake.setPower(0.5);
                TedMosby.intake.setMinPower(0.5);
            }),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.scoreGPP),

            TedMosby.shootOptimized(),

            new InstantCommand(() -> TedMosby.drivetrain.follower.setMaxPower(0.5)),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.intakePGP1),
            new InstantCommand(() -> TedMosby.intake.setPower(1)),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.intakePGP2),
            new WaitCommand(500),


            new InstantCommand(() -> {
                TedMosby.drivetrain.follower.setMaxPower(0.6);
                TedMosby.intake.setPower(0.5);
                TedMosby.intake.setMinPower(0.5);
            }),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.scorePGP),

            TedMosby.shootOptimized(),


            new InstantCommand(() -> {
                TedMosby.drivetrain.follower.setMaxPower(0.6);
                TedMosby.intake.setPower(0);
                TedMosby.intake.setMinPower(0);
            }),
            new FollowPathCommand(TedMosby.drivetrain.follower, paths.park)
        ));
    }


    @Override
    public void run() {
        super.run();
        TedMosby.update();
    }
}
