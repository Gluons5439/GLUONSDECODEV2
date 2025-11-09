package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-76.588468)
            .lateralZeroPowerAcceleration(-95.5170784)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.05,
                    0,
                    0.05,
                    0.02
            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.15,
                    0,
                    0.01,
                    0.02
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.027,
                    0,
                    0.00025,
                    0.6,
                    0.015
            ))
            .centripetalScaling(0.0004);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(82.22416194405143)
            .yVelocity(68.8235)
            .maxPower(1)
            .rightFrontMotorName("wheel3")
            .rightRearMotorName("wheel4")
            .leftRearMotorName("wheel1")
            .leftFrontMotorName("wheel2")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.1)
            .strafePodX(-2.7789866142)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}