package org.firstinspires.ftc.teamcode.scarlet.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.scarlet.Follower;
import org.firstinspires.ftc.teamcode.scarlet.utils.Pose;

@TeleOp
public class ForwardBack extends OpMode {
    Follower follower;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, new Pose());
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        if (gamepad1.triangle) follower.setTargetPose(new Pose(25, 0, 0));
        if (gamepad1.cross) follower.setTargetPose(new Pose(0, 0, 0));
        follower.updateConstants();
        follower.update();

        telemetry.addData("drive", follower.drive);
        telemetry.addData("strafe", follower.strafe);
        telemetry.addData("turn", follower.turn);
        telemetry.addData("Current", new Pose(follower.robotX, follower.robotY, follower.robotHeading).toString());
        telemetry.addData("Target", follower.targetPose.toString());

    }
}
