package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain extends SubsystemBase {
    public Follower follower;
    public double targetHeading;
    public Pose targetPose;
    public PIDFController headingController;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive(true);
        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        follower.update();
    }

    public void fieldCentric(Gamepad gamepad1) {
        follower.setTeleOpDrive(
                -gamepad1.right_stick_y,
                -gamepad1.right_stick_x,
                -gamepad1.left_stick_x,
                false
        );
        follower.update();
    }

    public void robotCentric(Gamepad gamepad1) {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();
    }

    public void setTargetHeading(double rad) {
        this.targetHeading = rad;
    } // use this method once when heading gamepad input is 0

    public void headingLock(Gamepad gamepad1, boolean robotCentric) {
        double error = targetHeading - follower.getHeading();
        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(error);

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                headingController.run(),
                robotCentric
        );
        follower.update();
    }

    public void savePose(Pose pose) {
        targetPose = pose;
    }

    public void holdPose() {
        follower.holdPoint(targetPose);
        follower.update();
    } // idk if you need to use headingLock() this might work

    public double getHeading() {
        return Math.toDegrees(follower.getHeading());
    }
}