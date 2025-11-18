package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.utils.Paths;
import org.firstinspires.ftc.teamcode.utils.Snoopy;
import org.firstinspires.ftc.teamcode.utils.commands.RunToVelocity;
import org.firstinspires.ftc.teamcode.utils.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.subsystems.Turret;

import java.util.Arrays;

@Autonomous
public class PedroTest extends CommandOpMode {

    Snoopy snoopy;
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Turret turret;

    Paths paths;

    @Override
    public void initialize() {
        snoopy = new Snoopy(hardwareMap, Snoopy.MatchState.AUTO, Snoopy.Alliance.BLUE);
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        paths = new Paths(drivetrain.follower);

        register(drivetrain, intake, shooter, turret);

        drivetrain.follower.setStartingPose(new Pose(24.000, 126.500, Math.toRadians(90)));
        schedule(new SequentialCommandGroup(
                new RunToVelocity(shooter, 500)
        ));
    }

    @Override
    public void run() {
        drivetrain.follower.update();
        telemetry.addData("isBusy", drivetrain.follower.isBusy()? "Busy" : "Not Busy");
        telemetry.update();
        super.run();
    }
}