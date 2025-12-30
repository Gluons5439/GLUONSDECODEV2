package org.firstinspires.ftc.teamcode.scarlet;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.scarlet.control.Controller;
import org.firstinspires.ftc.teamcode.scarlet.control.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.scarlet.control.PIDFController;
import org.firstinspires.ftc.teamcode.scarlet.control.PowerController;
import org.firstinspires.ftc.teamcode.scarlet.control.SquidController;
import org.firstinspires.ftc.teamcode.scarlet.utils.Pose;

public class Follower {
    public Pose targetPose = new Pose();

    private Controller headingController;
    private Controller translationalController;
    private Vector2d errorVector = new Vector2d();
    private double headingError;

    private final double RADIUS_THRESHOLD = 0.5;
    private final double HEADING_THRESHOLD = Math.toRadians(15);

    private GoBildaPinpointDriver pinpoint;
    public double robotX, robotY, robotHeading, robotVelX, robotVelY, robotVelTheta;

    public double turn, drive, strafe;

    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;

    public Follower(HardwareMap hMap, Pose start) {
        pinpoint = hMap.get(GoBildaPinpointDriver.class, ScarletConstants.pinpoint);
        pinpoint.setEncoderResolution(ScarletConstants.encoderRes);
        pinpoint.setOffsets(ScarletConstants.forwardY, ScarletConstants.strafeX, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(ScarletConstants.forwardPodDirection, ScarletConstants.strafePodDirection);

        headingController = ScarletConstants.headingController;
        translationalController = ScarletConstants.translationalController;

        frontLeft = new MotorEx(hMap, ScarletConstants.frontLeft);
        frontRight = new MotorEx(hMap, ScarletConstants.frontRight);
        backLeft = new MotorEx(hMap, ScarletConstants.backLeft);
        backRight = new MotorEx(hMap, ScarletConstants.backRight);

        frontLeft.setInverted(ScarletConstants.frontLeftDirection == MotorEx.Direction.REVERSE);
        frontRight.setInverted(ScarletConstants.frontRightDirection == MotorEx.Direction.REVERSE);
        backLeft.setInverted(ScarletConstants.backLeftDirection == MotorEx.Direction.REVERSE);
        backRight.setInverted(ScarletConstants.backRightDirection == MotorEx.Direction.REVERSE);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        pinpoint.setPosX(start.x, ScarletConstants.distanceUnit);
        pinpoint.setPosY(start.y, ScarletConstants.distanceUnit);
        pinpoint.setHeading(start.heading, ScarletConstants.angleUnit);

        targetPose.set(start.x, start.y, start.heading);
        robotX = start.x;
        robotY = start.y;
        robotHeading = start.heading;
    }

    public void update() {
        pinpoint.update();
        robotX = pinpoint.getPosX(ScarletConstants.distanceUnit);
        robotY = pinpoint.getPosY(ScarletConstants.distanceUnit);
        robotHeading = pinpoint.getHeading(ScarletConstants.angleUnit);
        robotVelX = pinpoint.getVelX(ScarletConstants.distanceUnit);
        robotVelY = pinpoint.getVelY(ScarletConstants.distanceUnit);
        robotVelTheta = pinpoint.getHeadingVelocity(ScarletConstants.angleUnit == AngleUnit.RADIANS ? UnnormalizedAngleUnit.RADIANS : UnnormalizedAngleUnit.DEGREES);

        calculateVectors();

        frontLeft.set(drive + strafe + turn);
        frontRight.set(drive - strafe - turn);
        backLeft.set(drive - strafe + turn);
        backRight.set(drive + strafe - turn);
    }

    public void calculateVectors() {
        headingError = AngleUnit.normalizeRadians(targetPose.heading - robotHeading);

        errorVector = new Vector2d(targetPose.x - robotX, targetPose.y - robotY);
        errorVector = errorVector.rotateBy(-robotHeading);

        turn = -headingController.calculate(headingError);
        drive = translationalController.calculate(errorVector.getX());
        strafe = -translationalController.calculate(errorVector.getY());

        if (drive > strafe && (Math.abs(drive) > 1 || Math.abs(strafe) > 1)) {
            strafe = strafe / drive;
            drive = 1;
        } else {
            drive = drive / strafe;
            strafe = 1;
        }

        drive *= Math.cos(Range.clip(headingError, -Math.PI / 2.0, Math.PI / 2.0));
        strafe *= Math.cos(Range.clip(headingError, -Math.PI / 2.0, Math.PI / 2.0));
    }


    public void updateConstants() {
        if (headingController instanceof FilteredPIDFController) {
            headingController.setConstants(ScarletConstants.headingP, ScarletConstants.headingI, ScarletConstants.headingD, ScarletConstants.headingF, ScarletConstants.headingAlpha);
        } else if (headingController instanceof PIDFController) {
            headingController.setConstants(ScarletConstants.headingP, ScarletConstants.headingI, ScarletConstants.headingD, ScarletConstants.headingF);
        } else if (headingController instanceof PowerController) {
            headingController.setConstants(ScarletConstants.headingPow, ScarletConstants.headingPower);
        } else if (headingController instanceof SquidController) {
            headingController.setConstants(ScarletConstants.headingSq);
        }

        if (translationalController instanceof FilteredPIDFController) {
            translationalController.setConstants(ScarletConstants.translatP, ScarletConstants.translatI, ScarletConstants.translatD, ScarletConstants.translatF, ScarletConstants.translatAlpha);
        } else if (translationalController instanceof PIDFController) {
            translationalController.setConstants(ScarletConstants.translatP, ScarletConstants.translatI, ScarletConstants.translatD, ScarletConstants.translatF);
        } else if (translationalController instanceof PowerController) {
            translationalController.setConstants(ScarletConstants.translatPow, ScarletConstants.translatPower);
        } else if (translationalController instanceof SquidController) {
            translationalController.setConstants(ScarletConstants.translatSq);
        }
    }

    public void setTargetPose(Pose pose) {
        targetPose = pose;
    }

    public boolean isInRadius(Pose pose, double radius) {
        return Math.hypot(pose.x - robotX, pose.y - robotY) < radius;
    }
    public boolean isAtTarget(double tolerance) {
        return isInRadius(targetPose, tolerance);
    }
}