package org.firstinspires.ftc.teamcode.scarlet;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.scarlet.control.Controller;
import org.firstinspires.ftc.teamcode.scarlet.control.FilteredPIDFController;

@Configurable
public class ScarletConstants {
    public static double headingPow = 0;
    public static double headingPower = 1;
    public static double headingSq = 0;
    public static double headingP = 1.5;
    public static double headingI = 0;
    public static double headingD = 0.1;
    public static double headingF = 0;
    public static double headingAlpha = 0.5;

    public static double translatPow = 0;
    public static double translatPower = 1;
    public static double translatSq = 0;
    public static double translatP = 0.001;
    public static double translatI = 0;
    public static double translatD = 0;
    public static double translatF = 0;
    public static double translatAlpha = 0.5;

    // change here to change controller type
    public static Controller headingController = new FilteredPIDFController(headingP, headingI, headingD, headingF, headingAlpha);
    public static Controller translationalController = new FilteredPIDFController(translatP, translatI, translatD, translatF, translatAlpha);

    public static final String frontLeft = "frontLeft";
    public static final String frontRight = "frontRight";
    public static final String backLeft = "backLeft";
    public static final String backRight = "backRight";
    public static final String pinpoint = "pinpoint";

    public static final Motor.Direction frontLeftDirection = Motor.Direction.FORWARD;
    public static final Motor.Direction frontRightDirection = Motor.Direction.REVERSE;
    public static final Motor.Direction backLeftDirection = Motor.Direction.FORWARD;
    public static final Motor.Direction backRightDirection = Motor.Direction.REVERSE;

    // you will have to retune if you change this
    public static final DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static final AngleUnit angleUnit = AngleUnit.RADIANS;

    public static final GoBildaOdometryPods encoderRes = GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final EncoderDirection strafePodDirection = EncoderDirection.FORWARD;
    public static final EncoderDirection forwardPodDirection = EncoderDirection.FORWARD;

    // use https://pedropathing.com/docs/pathing/tuning/localization/pinpoint#offsets to find offsets
    public static final double strafeX = -2.779;
    public static final double forwardY = 1.1;
}