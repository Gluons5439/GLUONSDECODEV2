package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.utils.subsystems.Shooter;

@TeleOp
@Configurable
public class VelocityTest extends CommandOpMode {
    Shooter shooter;
    Shooter hood;
    public static double velo;
    JoinedTelemetry tele;

    @Override
    public void initialize() {
        shooter = new Shooter(hardwareMap);
        tele = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        hood = new Shooter(hardwareMap);

    }

    @Override
    public void run() {
        shooter.setVelocity(velo);
        shooter.update();

        tele.addData("velocity", -shooter.shooter1.getCorrectedVelocity());
        tele.addData("raw velocity", -shooter.shooter1.encoder.getRawVelocity());
        tele.addData("target", velo);
        tele.addData("power", shooter.shooter2.get());
        tele.update();
    }
}