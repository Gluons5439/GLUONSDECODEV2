package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Stopper Test", group = "Testing")
@Configurable
public class HoodStopperTest extends CommandOpMode {

    // These will show up in the dashboard
    public static double hoodPosition = 0;
    public static double stopperPosition = 0.2;

    private Servo hood;
    private Servo stopper;

    @Override
    public void initialize() {
        hood = hardwareMap.get(Servo.class, "HoodServo");
        stopper = hardwareMap.get(Servo.class, "StopperServo");
    }

    @Override
    public void run() {
        // Apply live-configurable positions
        hood.setPosition(hoodPosition);
        stopper.setPosition(stopperPosition);

        // Telemetry for quick reference
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.addData("Stopper Pos", stopper.getPosition());
        telemetry.update();
    }
}
