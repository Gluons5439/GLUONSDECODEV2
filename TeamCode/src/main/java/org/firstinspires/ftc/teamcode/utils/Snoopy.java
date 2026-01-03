package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.utils.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.subsystems.Turret;

import java.util.concurrent.atomic.AtomicBoolean;

@Configurable
public class Snoopy {
    public enum MatchState {
        AUTO,
        TELEOP
    }
    public enum Alliance {
        RED,
        BLUE
    }

    public static final Pose BLUE_START_POSE = new Pose(24, 126.5, Math.toRadians(90));
    public static final Pose RED_START_POSE = new Pose(144-BLUE_START_POSE.getX(), BLUE_START_POSE.getY(), Math.toRadians(90));
    public static final Vector2d BLUE_GOAL = new Vector2d(0, 144);
    public static final Vector2d RED_GOAL = new Vector2d(144, 144);
    public static MatchState matchState;
    public static Alliance alliance;
    public static Drivetrain drivetrain;
    public static Turret turret;
    public static Intake intake;
    public static Shooter shooter;
    public static Pose startPose;
    public static Vector2d goal;

    public static int failsafeDelay = 100;
    public static int flywheelThreshhold = 80;
    public static double primeIntakeSpeed1 = 0.8;
    public static double primeIntakeSpeed2 = 0.5;
    public static double primeIntakeSpeed3 = 0;

    public static void init(HardwareMap hardwareMap, MatchState matchState, Alliance alliance) {
        Snoopy.matchState = matchState;
        Snoopy.alliance = alliance;
        Snoopy.startPose = alliance == Alliance.RED? RED_START_POSE : BLUE_START_POSE;
        Snoopy.goal = alliance == Alliance.RED? RED_GOAL : BLUE_GOAL;

        drivetrain = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        Storage.alliance = alliance;

        Snoopy.drivetrain.follower.setStartingPose(matchState == MatchState.AUTO ? startPose : Storage.pose);

        CommandScheduler.getInstance().registerSubsystem(drivetrain, turret, intake, shooter);

        CommandScheduler.getInstance().schedule(reset());
    }

    public static void update(){
        drivetrain.update();
        turret.update();
        intake.update();
        shooter.update();
    }

    public static InstantCommand reset() {
        return new InstantCommand(() -> {
            turret.enableAim = false;
            intake.setMinPower(0);
            shooter.setVelocity(Shooter.idleVeloMultiplier * Shooter.VELO_NEAR);
            shooter.closeStopper();
            shooter.resetHood();
        });
    }

    public static InstantCommand prime() {
        return new InstantCommand(() -> {
            turret.enableAim = true;
            intake.setMinPower(0);
            intake.setPower(0);
            shooter.setVelocity(Shooter.VELO_NEAR);
            shooter.closeStopper();
            shooter.raiseHood();
        });
    }

    public static Command shoot(){
        AtomicBoolean usedTimeout = new AtomicBoolean(false);
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> Snoopy.shooter.controller.atSetPoint()),
                new InstantCommand(() -> {
                    turret.enableAim = true;
                    intake.setMinPower(1);
                    shooter.setVelocity(Shooter.VELO_NEAR);
                    shooter.openStopper();
                    shooter.raiseHood();
                }),
                new WaitUntilCommand(() -> Math.abs(Snoopy.shooter.controller.getPositionError()) > flywheelThreshhold).raceWith(new WaitCommand(failsafeDelay).whenFinished(() -> usedTimeout.set(true))),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intake.setMinPower(-.5);
                                    intake.setPower(-.5);
                                }),
                                new WaitCommand(125),
                                new InstantCommand(() -> {
                                    intake.setPower(1);
                                    intake.setMinPower(1);
                                }),
                                new WaitCommand(750)
                        ),
                        new InstantCommand(),
                    usedTimeout::get
                )
        );
    }
    public static SequentialCommandGroup shootOptimized() {
        return new SequentialCommandGroup(
                prime(),
                shoot(),
                prime(),
                shoot(),
                prime(),
                shoot(),
                new WaitUntilCommand(() -> Snoopy.shooter.controller.atSetPoint()),
                reset()
        );
    };
}
