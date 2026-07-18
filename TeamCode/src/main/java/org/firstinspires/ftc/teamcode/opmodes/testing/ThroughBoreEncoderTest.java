package org.firstinspires.ftc.teamcode.opmodes.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Through Bore Encoder Test", group = "Calibration")
public class ThroughBoreEncoderTest extends LinearOpMode {

    private DcMotorEx shooterMotor2;

    @Override
    public void runOpMode() {
        /*
         * "shooterEncoder" must match the device name assigned to that
         * motor port in the Robot Configuration.
         *
         * If the encoder uses the shooter motor's encoder socket, use
         * the shooter's existing configuration name instead.
         */
        shooterMotor2 = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );
        //8192 ticks per revolution

        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Encoder initialized");
        telemetry.addLine("Press START, then rotate the shaft exactly once.");
        telemetry.update();

        waitForStart();

        int startingTicks = shooterMotor2.getCurrentPosition();
        boolean previousA = false;

        while (opModeIsActive()) {
            // Press A to set the current physical position as zero.
            if (gamepad1.a && !previousA) {
                startingTicks = shooterMotor2.getCurrentPosition();
            }
            previousA = gamepad1.a;

            int rawTicks = shooterMotor2.getCurrentPosition();
            int tickChange = rawTicks - startingTicks;

            telemetry.addData("Raw encoder ticks", rawTicks);
            telemetry.addData("Ticks since reset", tickChange);
            telemetry.addData("Absolute tick change", Math.abs(tickChange));
            telemetry.addLine();
            telemetry.addLine("Press A, rotate the encoder exactly once,");
            telemetry.addLine("then read Absolute tick change.");
            telemetry.update();
        }
    }
}