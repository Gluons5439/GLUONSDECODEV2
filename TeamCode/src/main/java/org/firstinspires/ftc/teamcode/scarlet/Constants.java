package org.firstinspires.ftc.teamcode.scarlet;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;

public class Constants {
    public static double kPow = 1;
    public static double kP = 0;
    public static double kD = 0;

    public static String frontLeft = "frontLeft";
    public static String frontRight = "frontRight";
    public static String backLeft = "backLeft";
    public static String backRight = "backRight";

    public static GoBildaOdometryPods encoderRes = GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static EncoderDirection strafePodDirection = EncoderDirection.FORWARD;
    public static EncoderDirection forwardPodDirection = EncoderDirection.FORWARD;;
    // units are in inches,
    // use https://pedropathing.com/docs/pathing/tuning/localization/pinpoint#offsets to find offsets
    public static double xOffset = 0;
    public static double yOffset = 0;
}