package frc.robot.subsystems.BallPath.Shooter;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Shooter extends Subsystem, LifecycleListener {
    void findAndCenterTarget();
    void centerTarget(double tx);
    void getDistance(double ty, double angle1, double angle2);
    boolean blocking();
    boolean readyToShoot();
    int checkBalls();
    void stopMotors();
    void setShotPosition(ShotPosition shotPosition);
    void resetSensors();

    enum ShotPosition {
        NONE,
        LAUNCHPAD_CLOSE,
        LAUNCHPAD_FAR,
        FENDER,
        TARMAC,
        TEST,
        TEST2,
        GENERAL
        ;
    }
}
