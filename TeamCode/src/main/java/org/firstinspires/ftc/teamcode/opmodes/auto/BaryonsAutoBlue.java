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
public class BaryonsAutoBlue extends CommandOpMode {

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
                    Mosby.intake.setPower(0);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakegpp1),
                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.intakegpp2),
                new WaitCommand(450),
                new InstantCommand(() -> Mosby.intake.setPower(0)),

                new InstantCommand(() -> {
                    Mosby.drivetrain.follower.setMaxPower(1);
                    //Mosby.intake.setPower(0.5);
                    //Mosby.intake.setMinPower(0.5);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.shootgpp),
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

                new InstantCommand(() -> Mosby.intake.setPower(1)),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.gatePickup),
                new WaitCommand(200),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.gateCheckBack),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.gateCheckForward),
                //new WaitCommand(200),
                new InstantCommand(() -> {
                    Mosby.intake.setPower(0);
                    Mosby.drivetrain.follower.setMaxPower(0.8);
                }),
                new FollowPathCommand(Mosby.drivetrain.follower, paths.shootGateCheck),
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
