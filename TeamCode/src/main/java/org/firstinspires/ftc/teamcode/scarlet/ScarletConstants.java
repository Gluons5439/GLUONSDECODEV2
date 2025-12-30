package org.firstinspires.ftc.teamcode.scarlet;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.scarlet.control.Controller;
import org.firstinspires.ftc.teamcode.scarlet.control.FilteredPIDFController;

@Configurable
public class Constants {
    public static double kPow = 0;

    public static double kSq = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double a = 0.5;

    // change here to change controller type
    public static Controller headingController = new FilteredPIDFController(kP, kI, kD, kF, a);
    public static Controller translationalController = new FilteredPIDFController(kP, kI, kD, kF, a);

    public static final String frontLeft = "frontLeft";
    public static final String frontRight = "frontRight";
    public static final String backLeft = "backLeft";
    public static final String backRight = "backRight";

    // you will have to retune if you change this
    public static final DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static final AngleUnit angleUnit = AngleUnit.RADIANS;

    public static final GoBildaOdometryPods encoderRes = GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final EncoderDirection strafePodDirection = EncoderDirection.FORWARD;
    public static final EncoderDirection forwardPodDirection = EncoderDirection.FORWARD;

    // use https://pedropathing.com/docs/pathing/tuning/localization/pinpoint#offsets to find offsets
    public static final double xOffset = 0;
    public static final double yOffset = 0;
}