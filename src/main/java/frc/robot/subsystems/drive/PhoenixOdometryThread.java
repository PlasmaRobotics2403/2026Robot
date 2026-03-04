package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

public class PhoenixOdometryThread extends Thread {
    private static final int WAIT_FOR_ALL_ERROR_THRESHOLD = 5;

    private final Lock signalsLock = new ReentrantLock();
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private boolean disableCanFdWaitForAll = false;
    private int consecutiveWaitForAllErrors = 0;

    private static boolean isCANFD = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD();
    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
    }

    @Override
    public void start() {
        if (timestampQueues.size() > 0) {
            super.start();
        }
    }

    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {

            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0 && !disableCanFdWaitForAll) {
                    var status = BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
                    if (status.isOK()) {
                        consecutiveWaitForAllErrors = 0;
                    } else {
                        consecutiveWaitForAllErrors++;
                        BaseStatusSignal.refreshAll(phoenixSignals);
                        if (consecutiveWaitForAllErrors >= WAIT_FOR_ALL_ERROR_THRESHOLD) {
                            disableCanFdWaitForAll = true;
                            DriverStation.reportWarning(
                                    "PhoenixOdometryThread: disabling CAN FD waitForAll after repeated stale CAN status frames. Falling back to refreshAll polling.",
                                    false);
                        }
                    }
                } else {
                    Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
                    if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            Drive.odometryLock.lock();
            try {
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                Drive.odometryLock.unlock();
            }
        }
    }
}
