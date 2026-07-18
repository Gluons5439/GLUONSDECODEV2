package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

@TeleOp(name = "Shooter PIDF Tuning", group = "Test")
@Configurable
public class ShooterPIDFTuning extends OpMode {

    private static final double THROUGH_BORE_TICKS_PER_REVOLUTION = 8192.0;
    private static final double SHOOTER_MAX_RPM = 6000.0;
    private static final long VELOCITY_SAMPLE_WINDOW_NANOS = 50_000_000L;
    private static final int VELOCITY_SAMPLE_CAPACITY = 128;

    // ======= TUNABLES (LIVE VIA PANELS) =======
    public static double P = 0.0011;
    public static double F = 0.00022;

    public static double TARGET_RPM = 1600;
    public static boolean SHOOTER_ON = false;
    public static double INTAKE_POWER = 0;

    // ==========================================

    private Motor shooterMotorWithThroughBore;
    private Motor secondShooterMotor;
    private Motor intakeMotor;

    private PIDFController controller;
    private int previousThroughBorePosition;
    private long previousThroughBoreSampleTimeNanos;
    private final int[] throughBoreSamplePositions =
            new int[VELOCITY_SAMPLE_CAPACITY];
    private final long[] throughBoreSampleTimesNanos =
            new long[VELOCITY_SAMPLE_CAPACITY];
    private int oldestThroughBoreSampleIndex;
    private int throughBoreSampleCount;
    private double filteredThroughBoreRpm;

    @Override
    public void init() {
        shooterMotorWithThroughBore = new Motor(
                hardwareMap,
                "shooterMotor",
                THROUGH_BORE_TICKS_PER_REVOLUTION,
                SHOOTER_MAX_RPM
        );
        secondShooterMotor = new Motor(
                hardwareMap,
                "shooterMotor2",
                Motor.GoBILDA.BARE
        );
        intakeMotor = new Motor(
                hardwareMap,
                "Intake",
                Motor.GoBILDA.RPM_1150
        );

        shooterMotorWithThroughBore.setInverted(true);
        shooterMotorWithThroughBore.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        secondShooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        controller = new PIDFController(P, 0, 0, F);
        controller.setTolerance(50);
        controller.setSetPoint(0);
    }

    @Override
    public void start() {
        previousThroughBorePosition = shooterMotorWithThroughBore.getCurrentPosition();
        previousThroughBoreSampleTimeNanos = System.nanoTime();
        oldestThroughBoreSampleIndex = 0;
        throughBoreSampleCount = 0;
        addThroughBoreSample(
                previousThroughBorePosition,
                previousThroughBoreSampleTimeNanos
        );
        filteredThroughBoreRpm = 0;
    }

    @Override
    public void loop() {
        controller.setP(P);
        controller.setF(F);

        int throughBorePosition = shooterMotorWithThroughBore.getCurrentPosition();
        long throughBoreSampleTimeNanos = System.nanoTime();
        double throughBoreTicksPerSecond = getThroughBoreTicksPerSecond(
                throughBorePosition,
                throughBoreSampleTimeNanos
        );
        double rawThroughBoreRpm =
                throughBoreTicksPerSecond * 60.0 / THROUGH_BORE_TICKS_PER_REVOLUTION;
        double throughBoreRpm = getFilteredThroughBoreRpm(
                throughBorePosition,
                throughBoreSampleTimeNanos
        );
        double rpmError = TARGET_RPM - throughBoreRpm;
        double power = 0;

        if (SHOOTER_ON) {
            controller.setSetPoint(TARGET_RPM);
            power = controller.calculate(throughBoreRpm);
            power = clamp(power, 0.0, 1.0);
        } else {
            controller.reset();
        }

        intakeMotor.set(INTAKE_POWER);
        setShooterPower(power);

        telemetry.addData("Shooter On", SHOOTER_ON);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Filtered Through Bore RPM", throughBoreRpm);
        telemetry.addData("Raw Through Bore RPM", rawThroughBoreRpm);
        telemetry.addData("RPM error (target - measured)", rpmError);
        telemetry.addData("RPM error magnitude", Math.abs(rpmError));
        telemetry.addData(
                "Through Bore position ticks",
                Math.abs((long) throughBorePosition)
        );
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Power Output", power);
        telemetry.update();
    }

    private double getThroughBoreTicksPerSecond(
            int currentPosition,
            long currentSampleTimeNanos
    ) {
        int positionChange = currentPosition - previousThroughBorePosition;
        long elapsedTimeNanos =
                currentSampleTimeNanos - previousThroughBoreSampleTimeNanos;

        previousThroughBorePosition = currentPosition;
        previousThroughBoreSampleTimeNanos = currentSampleTimeNanos;

        if (elapsedTimeNanos <= 0) {
            return 0;
        }

        return Math.abs(positionChange * 1_000_000_000.0 / elapsedTimeNanos);
    }

    private double getFilteredThroughBoreRpm(
            int currentPosition,
            long currentSampleTimeNanos
    ) {
        addThroughBoreSample(
                currentPosition,
                currentSampleTimeNanos
        );

        long windowStartTimeNanos =
                currentSampleTimeNanos - VELOCITY_SAMPLE_WINDOW_NANOS;

        while (throughBoreSampleCount > 1) {
            int nextSampleIndex =
                    (oldestThroughBoreSampleIndex + 1)
                            % VELOCITY_SAMPLE_CAPACITY;

            if (throughBoreSampleTimesNanos[nextSampleIndex]
                    > windowStartTimeNanos) {
                break;
            }

            oldestThroughBoreSampleIndex = nextSampleIndex;
            throughBoreSampleCount--;
        }

        long elapsedTimeNanos =
                currentSampleTimeNanos
                        - throughBoreSampleTimesNanos[oldestThroughBoreSampleIndex];

        if (elapsedTimeNanos < VELOCITY_SAMPLE_WINDOW_NANOS) {
            return filteredThroughBoreRpm;
        }

        int positionChange = currentPosition
                - throughBoreSamplePositions[oldestThroughBoreSampleIndex];
        filteredThroughBoreRpm = Math.abs(
                positionChange * 60_000_000_000.0
                        / elapsedTimeNanos
                        / THROUGH_BORE_TICKS_PER_REVOLUTION
        );
        return filteredThroughBoreRpm;
    }

    private void addThroughBoreSample(int position, long sampleTimeNanos) {
        if (throughBoreSampleCount == VELOCITY_SAMPLE_CAPACITY) {
            oldestThroughBoreSampleIndex =
                    (oldestThroughBoreSampleIndex + 1)
                            % VELOCITY_SAMPLE_CAPACITY;
            throughBoreSampleCount--;
        }

        int newSampleIndex =
                (oldestThroughBoreSampleIndex + throughBoreSampleCount)
                        % VELOCITY_SAMPLE_CAPACITY;
        throughBoreSamplePositions[newSampleIndex] = position;
        throughBoreSampleTimesNanos[newSampleIndex] = sampleTimeNanos;
        throughBoreSampleCount++;
    }

    private void setShooterPower(double power) {
        shooterMotorWithThroughBore.set(-power);
        secondShooterMotor.set(power);
    }

}