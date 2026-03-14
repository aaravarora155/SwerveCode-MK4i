package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.concurrent.locks.Lock;

public class PhoenixOdometryThread implements Runnable {
    private final Lock lock;
    private final List<Double> queue;

    public PhoenixOdometryThread(Lock lock, List<Double> queue) {
        this.lock = lock;
        this.queue = queue;
    }

    @Override
    public void run() {
        while (true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
