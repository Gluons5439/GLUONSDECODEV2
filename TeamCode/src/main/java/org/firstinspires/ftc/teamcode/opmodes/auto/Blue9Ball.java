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
public class Blue9Ball extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Mosby.init(hardwareMap, Mosby.MatchState.AUTO, Mosby.Alliance.BLUE);
        paths = new Paths(Mosby.drivetrain.follower, Mosby.Alliance.BLUE);
        Mosby.drivetrain.follower.setMaxPower(0.7);

        schedule(new SequentialCommandGroup(

            //
            new InstantCommand(() -> {
                Mosby.intake.setPower(0.5);
                Mosby.intake.setMinPower(0.5);
            }),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.startToScore),

            Mosby.shootOptimized(),

            new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(0.5)),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.intakeGPP1),
            new InstantCommand(() -> Mosby.intake.setPower(1)),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.intakeGPP2),
            new WaitCommand(500),

            new InstantCommand(() -> {
                Mosby.drivetrain.follower.setMaxPower(0.6);
                Mosby.intake.setPower(0.5);
                Mosby.intake.setMinPower(0.5);
            }),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.scoreGPP),

            Mosby.shootOptimized(),

            new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(0.5)),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.intakePGP1),
            new InstantCommand(() -> Mosby.intake.setPower(1)),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.intakePGP2),
            new WaitCommand(500),


            new InstantCommand(() -> {
                Mosby.drivetrain.follower.setMaxPower(0.6);
                Mosby.intake.setPower(0.5);
                Mosby.intake.setMinPower(0.5);
            }),
            new FollowPathCommand(Mosby.drivetrain.follower, paths.scorePGP),

            Mosby.shootOptimized(),


            new InstantCommand(() -> {
                Mosby.drivetrain.follower.setMaxPower(0.6);
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
    }
}
