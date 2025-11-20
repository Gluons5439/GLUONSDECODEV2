package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.utils.Snoopy;
import org.firstinspires.ftc.teamcode.utils.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.subsystems.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Configurable
public class TeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        Snoopy.init(hardwareMap, Snoopy.MatchState.TELEOP, Snoopy.Alliance.BLUE);
    }

    public void run() {
        Snoopy.update();

        if(gamepad1.cross){
            Snoopy.turret.setAngle(Math.toRadians(0));
        }else if(gamepad1.circle){
            Snoopy.turret.setAngle(Math.toRadians(90));
        }else if(gamepad1.triangle){
            Snoopy.turret.setAngle(Math.toRadians(180));
        }else if(gamepad1.square){
            Snoopy.turret.setAngle(Math.toRadians(270));
        }

        Turret.enableAim = gamepad1.dpad_up;

        Snoopy.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        Snoopy.drivetrain.drive(gamepad1);

        telemetry.update();
    }
}