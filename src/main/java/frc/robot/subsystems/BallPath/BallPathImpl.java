package frc.robot.subsystems.BallPath;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Elevator.Elevator.ElevatorAction;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.Intake.IntakeAction;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;

public class BallPathImpl extends RepeatingPooledSubsystem implements BallPath {

    private final Intake intake;
    private final Elevator elevator;
    private final Shooter shooter;

    private volatile BallAction action = BallAction.NONE;

    public BallPathImpl(Intake intake, Elevator elevator, Shooter shooter) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
    }

    @Override
    public void defineResources(){
        // require(intake);
        // require(elevator);
        // require(shooter);
    }

    @Override
    public void setAction(BallAction inputAction) {
        this.action = inputAction;
    }

    @Override
    public void task() {
        boolean intakeLoaded = this.intake.ballPrimed();
        boolean elevatorLoaded = this.elevator.ballPrimed();

        // boolean robotEmpty = !intakeLoaded && !elevatorLoaded;
        // boolean elevatorOnly = elevatorLoaded && !intakeLoaded;
        // boolean intakeOnly = intakeLoaded && !elevatorLoaded;
        // boolean full = intakeLoaded && elevatorLoaded;

        switch (action) {
            // case FEED:
            //     if (robotEmpty) {
            //         intake.setAction(IntakeAction.FEED);
            //         elevator.setAction(ElevatorAction.FEED);
            //         shooter.setShotPosition(ShotPosition.NONE);
            //     }
            //     if (elevatorOnly) {
            //         elevator.setAction(ElevatorAction.NONE);
            //         intake.setAction(IntakeAction.FEED);
            //         shooter.setShotPosition(ShotPosition.NONE);
            //     }
            //     if (intakeOnly) {
            //         elevator.setAction(ElevatorAction.FEED);
            //         intake.setAction(IntakeAction.PRIME);
            //         shooter.setShotPosition(ShotPosition.NONE);
            //     }
            //     if (full) {
            //         elevator.setAction(ElevatorAction.NONE);
            //         intake.setAction(IntakeAction.NONE);
            //         shooter.setShotPosition(ShotPosition.NONE);
            //     }
            //     break;
            // case SHOOT:
            //     if (robotEmpty) {
            //         elevator.setAction(ElevatorAction.FEED);
            //         intake.setAction(IntakeAction.NONE);
            //         shooter.setShotPosition(ShotPosition.NONE);
            //     }
            //     if (elevatorOnly) {
            //         if (shooter.readyToShoot()) {
            //             elevator.setAction(ElevatorAction.PRIME);
            //         }
            //         intake.setAction(IntakeAction.NONE);
            //     }
            //     if (intakeOnly) {
            //         elevator.setAction(ElevatorAction.PRIME);
            //         intake.setAction(IntakeAction.PRIME);
            //     }
            //     if (full) {
            //         if(shooter.readyToShoot()){
            //             elevator.setAction(ElevatorAction.PRIME);
            //             intake.setAction(IntakeAction.PRIME);
            //         }
            //     }
            //     break;
            case SHOOTGENERAL:
                this.shooter.setShotPosition(ShotPosition.GENERAL);
                if(shooter.readyToShoot()){
                    elevator.setAction(ElevatorAction.PRIME);
                }else{
                    elevator.setAction(ElevatorAction.NONE);
                }
                break;
            case SHOOTFENDER:
                this.shooter.setShotPosition(ShotPosition.FENDER);
                if(shooter.readyToShoot()){
                    elevator.setAction(ElevatorAction.PRIME);
                }else{
                    elevator.setAction(ElevatorAction.NONE);
                }
                break;
            case NONE:
                this.intake.setAction(IntakeAction.STOP);
                this.elevator.setAction(ElevatorAction.STOP);
                this.shooter.setShotPosition(ShotPosition.NONE);
                break;
            case INDEX:
                this.elevator.setAction(ElevatorAction.INDEX);
                this.intake.setAction(IntakeAction.IN);
                break;
            case OUT:
                this.elevator.setAction(ElevatorAction.OUT);
                this.intake.setAction(IntakeAction.OUT);
                break;
            case AUTO:
            case MANUAL:
                break;
            
            default:
                intake.setAction(IntakeAction.NONE);
                elevator.setAction(ElevatorAction.NONE);
                shooter.setShotPosition(ShotPosition.NONE);
                break;
        }
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_INIT:
            case ON_AUTO:
            case ON_TELEOP:
            case ON_TEST:
                this.start();
                break;
            case ON_DISABLED:
            case NONE:
            default:
                this.cancel();
                break;
        }
    }

    @Override
    public Intake getIntake(){
        return this.intake;
    }

    @Override
    public Elevator getElevator(){
        return this.elevator;
    }

    @Override
    public Shooter getShooter(){
        return this.shooter;
    }
}
