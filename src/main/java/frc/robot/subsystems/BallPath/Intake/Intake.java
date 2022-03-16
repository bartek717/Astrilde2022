package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Intake extends Subsystem, LifecycleListener {
    void setAction(IntakeAction action);
    boolean ballPrimed();

    enum IntakeAction {
        STOP,
        NONE,
        AUTO,
        PRIME,
        REJECT,
        FEED,
        TEST,
        IN,
        OUT,
        ;
    }
}
