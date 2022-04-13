package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Intake extends Subsystem, LifecycleListener {
    void setAction(IntakeAction action);
    boolean ballPrimed();

    enum IntakeAction {
        IN,
        OUT,
        STOP,
        INDEX,
        NONE;
    }
}
