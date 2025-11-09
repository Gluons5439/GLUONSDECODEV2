package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Intake extends SubsystemBase {
    public Motor intake;

    public double constantPower = 0.1;

    public Intake(HardwareMap hMap) {
        intake = new Motor(hMap, "intake");
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setInverted(true);
    }

    public void setPower(double power) {
        intake.set(power);
    }
}
