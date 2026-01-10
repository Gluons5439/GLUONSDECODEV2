package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WheelsTest extends OpMode {
    DcMotor wheel1;
    DcMotor wheel2;
    DcMotor wheel3;
    DcMotor wheel4;
    //I like this code

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        wheel1 = hardwareMap.get(DcMotor.class, "frontRightMotor");
        wheel2 = hardwareMap.get(DcMotor.class, "backRightMotor");
        wheel3 = hardwareMap.get(DcMotor.class, "backLeftMotor");
        wheel4 = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        if (gamepad1.cross) {
            wheel1.setPower(1);
        } else if (gamepad1.circle) {
            wheel2.setPower(1);
        } else if (gamepad1.square) {
            wheel3.setPower(1);
        } else if (gamepad1.triangle) {
            wheel4.setPower(1);
        } else {
            wheel1.setPower(0);
            wheel2.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
        }
    }
}
