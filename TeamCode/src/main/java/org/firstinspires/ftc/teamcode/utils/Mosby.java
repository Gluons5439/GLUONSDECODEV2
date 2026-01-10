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
public class Mosby {
    public enum MatchState {
        AUTO,
        TELEOP
    }
    public enum Alliance {
        RED,
        BLUE
    }

    public static final Pose BLUE_START_POSE = new Pose(64, 8, Math.toRadians(90));
    public static final Pose RED_START_POSE = new Pose(144-BLUE_START_POSE.getX(), BLUE_START_POSE.getY(), Math.toRadians(90));
    public static final Vector2d BLUE_GOAL = new Vector2d(16.5, 132);
    public static final Vector2d RED_GOAL = new Vector2d(127.5, 132);
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
        Mosby.matchState = matchState;
        Mosby.alliance = alliance;
        Mosby.startPose = alliance == Alliance.RED? RED_START_POSE : BLUE_START_POSE;
        Mosby.goal = alliance == Alliance.RED? RED_GOAL : BLUE_GOAL;

        drivetrain = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        Storage.alliance = alliance;

        Mosby.drivetrain.follower.setStartingPose(matchState == MatchState.AUTO ? startPose : Storage.pose);

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
            shooter.autoPower(false, false);
            shooter.setVelocity(Shooter.idleVeloMultiplier );
            shooter.closeStopper();
            //shooter.resetHood();1
        });
    }

    public static InstantCommand prime() {
        return new InstantCommand(() -> {

            turret.enableAim = true;

            intake.setMinPower(0);
            intake.setPower(0);

            shooter.openStopper();
            //shooter.raiseHood();

            shooter.autoPower(true, true);

        });
    }

    public static Command shoot(){
        AtomicBoolean usedTimeout = new AtomicBoolean(false);
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new InstantCommand(() -> {
                    turret.enableAim = true;

                    shooter.autoPower(true , true);
                    shooter.openStopper();
                    //shooter.raiseHood();
                }),
                new WaitUntilCommand(() -> Math.abs(Mosby.shooter.controller.getPositionError()) > flywheelThreshhold).raceWith(new WaitCommand(failsafeDelay).whenFinished(() -> usedTimeout.set(true))),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    shooter.hitTransfer();
                                }),
                                new WaitCommand(125),
                                new InstantCommand(() -> {
                                    shooter.downTransfer();
                                }),
                                new WaitCommand(125),
                                new InstantCommand(() -> {
                                    intake.setPower(1);
                                    intake.setMinPower(1);
                                }),
                                new WaitCommand(250),
                                new InstantCommand(() -> {
                                    intake.setPower(0);
                                    intake.setMinPower(0);
                                })
                        ),
                        new InstantCommand(),
                    usedTimeout::get
                )
        );
    }
    public static Command shootWithIntake(){
        AtomicBoolean usedTimeout = new AtomicBoolean(false);
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                new InstantCommand(() -> {
                    turret.enableAim = true;

                    shooter.autoPower(true, true);
                    shooter.openStopper();
                    //shooter.raiseHood();
                }),
                new WaitUntilCommand(() -> Math.abs(Mosby.shooter.controller.getPositionError()) > flywheelThreshhold).raceWith(new WaitCommand(failsafeDelay).whenFinished(() -> usedTimeout.set(true))),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    intake.setPower(1);
                                    intake.setMinPower(1);
                                }),
                                new WaitCommand(100),
                                new InstantCommand(() -> {
                                    shooter.autoPower(true, false);
                                    shooter.setHoodPercent(shooter.hood.getRawPosition()-0.05);
                                }),
                                new WaitCommand(100),
                                new InstantCommand(() -> {
                                    shooter.setHoodPercent(shooter.hood.getRawPosition()-0.05);
                                }),
                                new WaitCommand(50),
                                new InstantCommand(() -> {
                                    shooter.hitTransfer();
                                }),
                                new WaitCommand(200),
                                new InstantCommand(() -> {
                                    shooter.downTransfer();
                                    intake.setPower(0);
                                    intake.setMinPower(0);
                                    reset();
                                })
                        ),
                        new InstantCommand(),
                        usedTimeout::get
                )
        );
    }






/**
    public static Command shoot() {
        return new InstantCommand(() -> {



        });
    }
 **/

    public static SequentialCommandGroup shootOptimized() {
        return new SequentialCommandGroup(
                prime(),
                shoot(),
                prime(),
                shoot(),
                prime(),
                shoot(),
                new WaitUntilCommand(() -> Mosby.shooter.controller.atSetPoint()),
                reset()
        );
    };
}
