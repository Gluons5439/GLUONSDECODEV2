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
public class BlueOPT12 extends CommandOpMode {

    Paths paths;

    @Override
    public void initialize() {
        Mosby.init(hardwareMap, Mosby.MatchState.AUTO, Mosby.Alliance.BLUECLOSE);
        paths = new Paths(Mosby.drivetrain.follower, Mosby.Alliance.BLUECLOSE);
        Mosby.drivetrain.follower.setMaxPower(0.8);


        schedule(new SequentialCommandGroup(

                new FollowPathCommand(Mosby.drivetrain.follower, paths.ClosestartToScore),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),


                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(0.8);
                    Mosby.intake.setPower(1);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CloseintakePPG),
                new InstantCommand(() -> Mosby.intake.setPower(0)),


                new FollowPathCommand(Mosby.drivetrain.follower, paths.ClosescorePPG),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),


                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    Mosby.intake.setPower(1);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CloseintakePGP1),
                new WaitCommand(100),
                new InstantCommand(() -> Mosby.intake.setPower(0)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.turn1),


                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.ClosescorePGP),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),


                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    Mosby.intake.setPower(1);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CloseintakeGPP1),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.CloseintakeGPP2),
                new WaitCommand(400),
                new InstantCommand(() -> Mosby.intake.setPower(0)),


                new InstantCommand(() -> Mosby.drivetrain.follower.setMaxPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.ClosescoreGPP),
                Mosby.prime(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                Mosby.shootWithIntake(),
                Mosby.reset(),


                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    Mosby.intake.setPower(0);
                    Mosby.intake.setMinPower(0);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.Closepark)
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
