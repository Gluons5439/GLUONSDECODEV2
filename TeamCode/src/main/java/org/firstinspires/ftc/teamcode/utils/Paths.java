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
    public PathChain intakePPG1;
    public PathChain intakePPG2;
    public PathChain scorePPG;
    public PathChain park;
    public Pose startPose;
    public Pose shootingPose = new Pose(67,19, Math.toRadians(120));
    public Pose intakeGPP1Pose = new Pose(42, 35, shootingPose.getHeading());
    public Pose intakeGPP2Pose = new Pose(21.5, 35, shootingPose.getHeading());
    public Pose openGatePose = new Pose(14, 75, shootingPose.getHeading());
    public Pose openGateControl = new Pose(28,77);
    public Pose intakePGP1Pose = new Pose(43.5, 60, shootingPose.getHeading());
    public Pose intakePGP2Pose = new Pose(20, 60, shootingPose.getHeading());
    public Pose scorePGPControl = new Pose(40, 55);
    public Pose intakePPG1Pose = new Pose(42,84, shootingPose.getHeading());
    public Pose intakePPG2Pose = new Pose(18.5, 84, shootingPose.getHeading());
    public Pose intakePPG1Control = new Pose(57, 36);
    public Pose parkPose = new Pose(42, 19, shootingPose.getHeading());

    public Paths(Follower follower, Mosby.Alliance alliance) {
        startPose = Mosby.startPose;
        if (alliance == Mosby.Alliance.RED) {
            shootingPose = shootingPose.mirror();
            intakeGPP1Pose = intakeGPP1Pose.mirror();
            intakeGPP2Pose = intakeGPP2Pose.mirror();
            openGatePose = openGatePose.mirror();
            intakePGP1Pose = intakePGP1Pose.mirror();
            intakePGP2Pose = intakePGP2Pose.mirror();
            intakePPG1Pose = intakePPG1Pose.mirror();
            intakePPG2Pose = intakePPG2Pose.mirror();
            parkPose = parkPose.mirror();
            openGateControl = openGateControl.mirror();
            scorePGPControl = scorePGPControl.mirror();
            intakePPG1Control = intakePPG1Control.mirror();
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

        intakePPG1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(shootingPose, intakePPG1Pose)
                )
                .setBrakingStrength(3.5)
//                .setTimeoutConstraint(500)
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        intakePPG2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intakePPG1Pose, intakePPG2Pose)
                )
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        scorePPG = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(intakePPG2Pose, scorePGPControl, shootingPose)
                )
                .setBrakingStrength(3.5)
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