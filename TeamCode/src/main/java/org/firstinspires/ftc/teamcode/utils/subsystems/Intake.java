package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Intake extends SubsystemBase {
    public Motor intake;

    public double minPower = 0;
    private double power = minPower;

    public Intake(HardwareMap hMap) {
        intake = new Motor(hMap, "intake");
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setInverted(true);
        intake.set(minPower);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setMinPower(double minPower){
        this.minPower = minPower;
    }

    public void update(){
        double pow = Math.max(Math.abs(power), Math.abs(minPower));
        if(pow == Math.abs(minPower)){
            intake.set(minPower);
        }else {
            intake.set(power);
        }

    }
}
