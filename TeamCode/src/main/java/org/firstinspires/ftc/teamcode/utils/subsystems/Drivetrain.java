package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.Storage;

@Configurable
public class Drivetrain extends SubsystemBase {
    public Follower follower;
    public static double slow = .2;
    public PIDFController headingController;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        follower.update();
    }

    public void drive(Gamepad gamepad1) {
        double multiplier = gamepad1.left_bumper? slow : 1;
        follower.setTeleOpDrive(
                -gamepad1.right_stick_y * multiplier,
                -gamepad1.right_stick_x * multiplier,
                -gamepad1.left_stick_x * multiplier,
                true
        );
    }

    public void update(){
        follower.update();
        Storage.pose = follower.getPose();
    }
}