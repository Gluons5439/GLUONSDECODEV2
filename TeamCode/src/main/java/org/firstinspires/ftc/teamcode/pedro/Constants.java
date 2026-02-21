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
            .mass(12.247)
            .forwardZeroPowerAcceleration(-39.71745564498387)//-76.
            .lateralZeroPowerAcceleration(-49.43798150797455)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.9,
                    0,
                    0.00001,
                    0.026
            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0.0002,
                    0.026
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.017,
                    0,
                    0.0004,
                    0.6,
                    0.022
            ))
            .centripetalScaling(0.00045)
            .automaticHoldEnd(true);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.28,
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
            .xVelocity(81.88966477762058)
           .yVelocity(66.4859253890871)
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(false);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.5625)
            .strafePodX(-5.84375)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}