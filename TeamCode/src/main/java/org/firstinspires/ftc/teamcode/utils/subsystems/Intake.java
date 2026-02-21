package org.firstinspires.ftc.teamcode.utils.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
/*
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

*/


public class Intake extends SubsystemBase {

    public Motor intake;

    // Same behavior as your old intake
    private static final double INTAKE_POWER = -1.0;
    private static final double OUTTAKE_POWER = 0.25;

    // Internal state
    private double minPower = 0.0;
    private double power = 0.0;

    public Intake(HardwareMap hMap) {
        intake = new Motor(hMap, "Intake");

        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setInverted(true); // matches old negative power logic
        intake.set(minPower);
    }

    public void setPower(double power)
    {
        this.power = power;
    }
    public void setPower(double power, double time) {

    }


    public void forceIntake() {
        power = INTAKE_POWER;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }



    public void periodic() {
        double pow = Math.max(Math.abs(power), Math.abs(minPower));

        if (pow == Math.abs(minPower)) {
            intake.set(minPower);
        } else {
            intake.set(power);
        }
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
