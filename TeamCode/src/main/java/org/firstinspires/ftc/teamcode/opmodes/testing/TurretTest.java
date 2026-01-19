package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Encoder Read", group = "Testing")
public class TurretTest extends OpMode {

    private DcMotor turret;
    private int startTicks;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turretMotor");

        // Reset encoder
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Let it spin freely
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        startTicks = 0;

        telemetry.addLine("Rotate turret by hand");
    }

    @Override
    public void loop() {
        int currentTicks = turret.getCurrentPosition();
        int deltaTicks = currentTicks - startTicks;

        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Delta Ticks", deltaTicks);
        telemetry.update();
    }
}