package frc.robot.subsystems.Climber;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

// import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Climber extends Subsystem, LifecycleListener {
    void extendOuterClimber(double speed);
    void retractOuterClimber(double speed);
    // to attach inner, non retractable arm to the bar
    void extendShoulderLifter(double speed);
    void retractInnerLifter(double speed);
    void angleOuter(double angle);
}
