package frc.robot.subsystems.BallPath;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Shooter.Shooter;

public interface BallPath extends Subsystem, LifecycleListener {
    void setAction(BallAction action);
    Intake getIntake();
    Shooter getShooter();
    Elevator getElevator();

    enum BallAction {
        NONE,
        AUTO,
        FEED,
        SHOOTGENERAL,
        SHOOTFENDER,
        TEST,
        MANUAL,
        INDEX,
        OUT,
        NO_SHOOT,
        YES_SHOOT,
        ;
    }
}
