package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class ServoTest extends OpMode {
    Servo servo;
    Servo servo2;
    public static double pos = 0;
    public static double pos2=0;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "StopperServo");
        servo2 = hardwareMap.get(Servo.class, "HoodServo");
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
        servo2.setPosition(pos2);
    }
}
