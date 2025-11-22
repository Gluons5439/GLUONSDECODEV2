package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.Snoopy;

@Autonomous
public class BlueAuto extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Snoopy.init(hardwareMap, Snoopy.MatchState.AUTO, Snoopy.Alliance.BLUE);
        paths = new Paths(Snoopy.drivetrain.follower, Snoopy.Alliance.BLUE);
        Snoopy.drivetrain.follower.setMaxPower(0.7);

        schedule(new SequentialCommandGroup(

            new FollowPathCommand(Snoopy.drivetrain.follower, paths.startToScore),

            Snoopy.shootOptimized(),

            new InstantCommand(() -> Snoopy.drivetrain.follower.setMaxPower(0.5)),
            new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakeGPP1),
            new InstantCommand(() -> Snoopy.intake.setPower(1)),
            new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakeGPP2),
            new WaitCommand(500),

            new InstantCommand(() -> Snoopy.drivetrain.follower.setMaxPower(0.6)),
            new FollowPathCommand(Snoopy.drivetrain.follower, paths.scoreGPP),

            Snoopy.shootOptimized(),

            new InstantCommand(() -> Snoopy.drivetrain.follower.setMaxPower(0.5)),
            new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakePGP1),
            new InstantCommand(() -> Snoopy.intake.setPower(1)),
            new FollowPathCommand(Snoopy.drivetrain.follower, paths.intakePGP2),
            new WaitCommand(500),


            new InstantCommand(() -> Snoopy.drivetrain.follower.setMaxPower(0.6)),
            new FollowPathCommand(Snoopy.drivetrain.follower, paths.scorePGP),

            Snoopy.shootOptimized(),


            new InstantCommand(() -> {
                Snoopy.drivetrain.follower.setMaxPower(0.6);
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
    }
}
