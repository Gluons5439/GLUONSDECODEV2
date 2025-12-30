package org.firstinspires.ftc.teamcode.scarlet.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.scarlet.Follower;
import org.firstinspires.ftc.teamcode.scarlet.utils.Pose;

@TeleOp
public class TurnTuner extends OpMode {
    Follower follower;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, new Pose(0,0, Math.PI/2.0));
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        if (gamepad1.dpad_right) follower.setTargetPose(new Pose(follower.robotX, follower.robotY, 0));
        if (gamepad1.dpad_left) follower.setTargetPose(new Pose(follower.robotX, follower.robotY, Math.PI/2.0));
        if (gamepad1.circle) follower.setTargetPose(new Pose(follower.robotX, follower.robotY, Math.PI));
        if (gamepad1.square) follower.setTargetPose(new Pose(follower.robotX, follower.robotY, 1.5 * Math.PI));

//        follower.robotHeading = Range.clip(follower.robotHeading, 0, 2 * Math.PI);

        if (gamepad1.ps) follower.setTargetPose(new Pose(0,0,0));

        follower.updateConstants();
        follower.update();

        telemetry.addData("drive", follower.drive);
        telemetry.addData("strafe", follower.strafe);
        telemetry.addData("turn", follower.turn);
        telemetry.addData("Current", new Pose(follower.robotX, follower.robotY, follower.robotHeading).toString());
        telemetry.addData("Target", follower.targetPose.toString());
    }
}
