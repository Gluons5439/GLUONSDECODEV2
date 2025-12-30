package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    public PathChain startToScore;
    public PathChain intakeGPP1;
    public PathChain intakeGPP2;
    public PathChain openGate;
    public PathChain scoreGPP;
    public PathChain intakePGP1;
    public PathChain intakePGP2;
    public PathChain scorePGP;
    public PathChain intakePPG;
    public PathChain scorePPG;
    public PathChain park;

    public Pose startPose;
    public Pose shootingPose = new Pose(32,112, Math.toRadians(180));
    public Pose intakeGPP1Pose = new Pose(45, 84, shootingPose.getHeading());
    public Pose intakeGPP2Pose = new Pose(17, 84, shootingPose.getHeading());
    public Pose openGatePose = new Pose(14, 75, shootingPose.getHeading());
    public Pose openGateControl = new Pose(28,77);
    public Pose intakePGP1Pose = new Pose(45, 60, shootingPose.getHeading());
    public Pose intakePGP2Pose = new Pose(8.5, 60, shootingPose.getHeading());
    public Pose scorePGPControl = new Pose(40, 55);
    public Pose intakePPGPose = new Pose(48,39, shootingPose.getHeading());
    public Pose intakePPGControl1 = new Pose(36, 22);
    public Pose intakePPGControl2 = new Pose(71, 37);
    public Pose parkPose = new Pose(22, 72, Math.toRadians(90));

    public Paths(Follower follower, Snoopy.Alliance alliance) {
        startPose = Snoopy.startPose;
        if (alliance == Snoopy.Alliance.RED) {
            shootingPose = shootingPose.mirror();
            intakeGPP1Pose = intakeGPP1Pose.mirror();
            intakeGPP2Pose = intakeGPP2Pose.mirror();
            openGatePose = openGatePose.mirror();
            openGateControl = openGateControl.mirror();
            intakePGP1Pose = intakePGP1Pose.mirror();
            intakePGP2Pose = intakePGP2Pose.mirror();
            intakePPGPose = intakePPGPose.mirror();
            intakePPGControl1 = intakePPGControl1.mirror();
            intakePPGControl2 = intakePPGControl2.mirror();
            scorePGPControl = scorePGPControl.mirror();
            parkPose = parkPose.mirror();
        }

        startToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, shootingPose)
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        intakeGPP1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, intakeGPP1Pose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        intakeGPP2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intakeGPP1Pose, intakeGPP2Pose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        scoreGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intakeGPP2Pose, shootingPose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        intakePGP1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, intakePGP1Pose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        intakePGP2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intakePGP1Pose, intakePGP2Pose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        scorePGP = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakePGP2Pose,
                                scorePGPControl,
                                shootingPose
                        )
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        intakePPG = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(shootingPose, intakePPGControl1, intakePPGControl2, intakePPGPose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();



        scorePPG = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(intakePPGPose, scorePGPControl,shootingPose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, parkPose)
                )
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

    }
}