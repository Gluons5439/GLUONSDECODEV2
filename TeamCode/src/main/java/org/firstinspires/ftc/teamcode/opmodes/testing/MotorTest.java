package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class MotorTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    Servo hood;
    public static double pos = 0.2;

     //I like this code

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "shooterMotor");
        motor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "stopper");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        double power = gamepad1.right_trigger - gamepad1.left_trigger;

        hood.setPosition(pos);
        motor1.setPower(power);
        motor2.setPower(power);
        telemetry.addData("Power", power);
        telemetry.update();

    }
}
