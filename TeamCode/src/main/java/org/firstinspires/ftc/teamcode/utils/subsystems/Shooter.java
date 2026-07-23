
package org.firstinspires.ftc.teamcode.utils.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.utils.Mosby;
import org.firstinspires.ftc.teamcode.utils.Storage;


@Configurable
public class Shooter extends SubsystemBase {
    // shooter1's Through Bore encoder is 1:1 with the shooter: 8,192 ticks equals one revolution.
    private static final double THROUGH_BORE_TICKS_PER_REVOLUTION = 8192.0;
    private static final double SHOOTER_MAX_RPM = 6000.0;

    // Calculate speed over at least 50 ms to reject single-loop encoder spikes.
    private static final long VELOCITY_SAMPLE_WINDOW_NANOS = 50_000_000L;
    private static final int VELOCITY_SAMPLE_CAPACITY = 128;

    public Motor shooter1;
    public Motor shooter2;
    public ServoEx hood;
    public ServoEx stopper;
    public ServoEx transfer;

    public static double P = 0.0011;//0.006 0.000389
    public static double D = 0.0;
    public static double F = 0.00022;//0.0008
    public PIDFController controller = new PIDFController(P, 0, D, F);
    public static double TOLERANCE = 50;

    public static double STOPPER_OPEN = 0.1;
    public static double STOPPER_CLOSED = 0.3;
    public static double TRANSFER_UP = 0.85;
    public static double TRANSFER_DOWN = 0.5;
    public static double HOOD_MIN = 0;
    public static double HOOD_MAX = 1;

    public static double HOOD_NEAR = 0;


    public static double idleVeloMultiplier = 0.0;

    public static double multiplier = 0.65;

    InterpLUT lutVelocity = new InterpLUT();
    InterpLUT lutHood = new InterpLUT();
    public double distance;
    public double power;
    public boolean shooterBlah;
    public Pose pos;

    // Fixed-size ring buffer keeps recent encoder samples without allocating objects every loop.
    private final int[] throughBoreSamplePositions =
            new int[VELOCITY_SAMPLE_CAPACITY];
    private final long[] throughBoreSampleTimesNanos =
            new long[VELOCITY_SAMPLE_CAPACITY];
    private int oldestThroughBoreSampleIndex;
    private int throughBoreSampleCount;
    private double filteredThroughBoreRpm;

    // Idle mode bypasses the distance LUT and holds a fixed RPM supplied by the caller.
    private boolean idleMode;
    private double idleTargetRpm;

    // Business mode holds a fixed hood position instead of using the distance LUT.
    private boolean businessMode;
    private double businessHoodPosition;
    //double currentVelocity = 0;

    public Shooter(HardwareMap hMap) {
        // shooter1 reads the Through Bore encoder; shooter2 mirrors the calculated motor power.
        shooter1 = new Motor(
                hMap,
                "shooterMotor",
                THROUGH_BORE_TICKS_PER_REVOLUTION,
                SHOOTER_MAX_RPM
        );
        shooter2 = new Motor(hMap, "shooterMotor2", Motor.GoBILDA.BARE);
        //shooter1.setInverted(true);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);


        hood = new ServoEx(hMap, "HoodServo");
        stopper = new ServoEx(hMap, "StopperServo");
        transfer = new ServoEx(hMap, "TransferServo");
        transfer.set(TRANSFER_DOWN);

        shooter1.setInverted(true);
        controller.setTolerance(TOLERANCE);
        controller.setSetPoint(0);
        // All shooter LUT outputs are physical RPM, matching the PIDF setpoint and encoder measurement.
        // These were converted from the old 28-tick motor velocity units.
        lutVelocity.add(-1000, 2977);
        lutVelocity.add(0, 2978.571);
        lutVelocity.add(30.5, 3042.857);
        lutVelocity.add(39.5, 3085.714);
        lutVelocity.add(48.5, 3171.429);
        lutVelocity.add(58.5, 3300.0);
        lutVelocity.add(71.5, 3300.0);
        lutVelocity.add(86.5, 3514.286);
        lutVelocity.add(96.5, 3600.0);
        lutVelocity.add(111.5, 3857.143);
        lutVelocity.add(122.5, 4028.571);
        lutVelocity.add(142.5, 4200.0);
        lutVelocity.add(200, 4457.143);
        lutVelocity.add(1000,4458);

        lutHood.add(-1000,0);
        lutHood.add(0, 0);
        lutHood.add(30.5, 0);
        lutHood.add(39.5, 0);
        lutHood.add(48.5, 0.05);
        lutHood.add(58.5, 0.1);
        lutHood.add(71.5, 0.22);
        lutHood.add(86.5, 0.26);
        lutHood.add(96.5, 0.26);
        lutHood.add(111.5, 0.28);
        lutHood.add(122.5, 0.3);
        lutHood.add(142.5, 0.33);
        lutHood.add(200, 0.35);
        lutHood.add(1000,0.36);


        //Velcoity for buisness
        /**
         lutVelocity.add(-200, 3600.0);
         lutVelocity.add(1000, 3600.0);
         lutHood.add(-200, 0.26);
         lutHood.add(1000, 0.26);
         **/

        lutVelocity.createLUT();
        lutHood.createLUT();
        pos = Mosby.drivetrain.follower.getPose();
        controller.setP(P);
        controller.setF(F);


        shooterBlah = false;
        distance = Math.hypot(Mosby.goalShooter.getX()-Storage.pose.getX(),Mosby.goalShooter.getY()-Storage.pose.getY());


    }

    public void update() {
        // Sample even while stopped so the rolling RPM history is ready before spin-up.
        sampleThroughBoreRpm();

        // Reapply tunable gains so dashboard changes take effect without restarting the OpMode.
        controller.setP(P);
        controller.setF(F);

        // Recalculate distance and command the hood every loop, including while stopped or idling.
        pos = Mosby.drivetrain.follower.getPose();
        distance = Math.hypot(
                Mosby.goalShooter.getX() - pos.getX(),
                Mosby.goalShooter.getY() - pos.getY()
        );
        if (businessMode) {
            hood.set(businessHoodPosition);
        } else {
            hood.set(lutHood.get(distance));
        }

        double targetRpm;

        // Shooting uses the distance LUT; idle mode uses the caller's fixed RPM target.
        if (shooterBlah) {
            targetRpm = lutVelocity.get(distance);
        } else if (idleMode) {
            targetRpm = idleTargetRpm;
        } else {
            controller.setSetPoint(0);
            setPower(0);
            return;
        }

        controller.setSetPoint(targetRpm);

        // The PIDF setpoint and measurement are both RPM; output is safe forward motor power.
        power = clamp(controller.calculate(filteredThroughBoreRpm), 0.0, 1.0);
        setPower(power);
    }

    /**
     * Updates filtered RPM from Through Bore position change across a rolling time window.
     * Keeping a sample at or before the 50 ms boundary prevents short-loop timing jitter
     * from producing the false 700/2,000 RPM flashes seen with instantaneous velocity.
     */
    private void sampleThroughBoreRpm() {
        int currentPosition = shooter1.getCurrentPosition();
        long currentSampleTimeNanos = System.nanoTime();
        addThroughBoreSample(currentPosition, currentSampleTimeNanos);

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
            return;
        }

        int positionChange = currentPosition
                - throughBoreSamplePositions[oldestThroughBoreSampleIndex];

        // Absolute position change reports speed in RPM regardless of encoder direction.
        filteredThroughBoreRpm = Math.abs(
                positionChange * 60_000_000_000.0
                        / elapsedTimeNanos
                        / THROUGH_BORE_TICKS_PER_REVOLUTION
        );
    }

    /** Adds one position/time sample and overwrites the oldest entry only if the buffer fills. */
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


    public void setShooter(boolean s) {
        businessMode = false;
        // Explicit shooter commands leave fixed-RPM idle mode.
        idleMode = false;
        shooterBlah = s;
    }

    /**
     * Holds a fixed idle speed with the filtered Through Bore RPM and PIDF controller.
     * Call autoPower(...) or setShooter(...) to leave idle mode.
     *
     * @param rpm requested idle speed, clamped from 0 to SHOOTER_MAX_RPM
     */
    public void runIdle(double rpm) {
        businessMode = false;
        shooterBlah = false;
        idleMode = true;
        idleTargetRpm = clamp(rpm, 0.0, SHOOTER_MAX_RPM);
    }

    /** Holds a fixed RPM and hood position for close-range business shots. */
    public void runBusinessMode(double rpm, double hoodPosition) {
        shooterBlah = false;
        idleMode = true;
        idleTargetRpm = clamp(rpm, 0.0, SHOOTER_MAX_RPM);
        businessMode = true;
        businessHoodPosition = clamp(hoodPosition, HOOD_MIN, HOOD_MAX);
    }

    public void setVelocity(double velocity) {
        controller.setSetPoint(velocity);
        // currentVelocity = velocity;
    }

    /** Returns the latest filtered Through Bore speed in RPM. */
    public double getVelocity() {
        return filteredThroughBoreRpm;
    }

    public void setPower(double power) {
        // Do not allow a PID overshoot to reverse the flywheel motors.
        power = clamp(power, 0.0, 1.0);
        shooter1.set(-power);
        shooter2.set(power);
    }

    public void autoPower(boolean shooterOn, boolean hoodOn) {
        businessMode = false;
        // Switching to normal shooting or off cancels fixed-RPM idle mode.
        idleMode = false;
        shooterBlah = shooterOn;

        if (shooterOn) {
            controller.setP(P);
            controller.setF(F);
            controller.setSetPoint(lutVelocity.get(distance) );
        } else {
            controller.setSetPoint(0);
        }

        if (hoodOn) {
            hood.set(lutHood.get(distance));
        }
    }


    public void closeStopper() {
        stopper.set(STOPPER_CLOSED);
    }

    public void openStopper() {
        stopper.set(STOPPER_OPEN);
    }
    public void hitTransfer(){
        transfer.set(TRANSFER_UP);
    }
    public void downTransfer(){
        transfer.set(TRANSFER_DOWN);
    }

    public void resetHood() {
        setHoodPercent(0);
    }

    public void raiseHood() {
        setHoodPercent(HOOD_NEAR);
    }

    public void setHoodPercent(double percent) {
        hood.set( HOOD_MAX * percent);
    }
    public void setCurrentHoodPercent(double percent) {hood.set(hood.getRawPosition()*percent);}
}
