package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Intake extends Subsystem, LifecycleListener {
    void setAction(IntakeAction action);
    boolean ballPrimed();
    int getBallsIntake();

    enum IntakeAction {
        STOP,
        NONE,
        AUTO,
        PRIME,
        REJECT,
        FEED,
        TESTIN,
        TESTOUT,
        TESTSHOOT,
        IN,
        OUT,
        ;
    }
}
