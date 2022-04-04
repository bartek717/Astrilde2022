package frc.robot.subsystems.BallPath;

import java.util.concurrent.TimeUnit;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Elevator.ElevatorImpl;
import frc.robot.subsystems.BallPath.Elevator.Elevator.ElevatorAction;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.Intake.IntakeAction;
import frc.robot.subsystems.BallPath.Shooter.PIDShooterTrackingImpl;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;

public class BallPathImpl extends RepeatingPooledSubsystem implements BallPath {

    private final Intake intake;
    private final Elevator elevator;
    private final Shooter shooter;
    private Spark blinkenController;
    boolean checkBall = false;
    boolean noShoot = false;

    private volatile BallAction action = BallAction.NONE;

    public BallPathImpl(Intake intake, Elevator elevator, Shooter shooter, Spark blinkenController) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
        this.blinkenController = blinkenController;
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


        switch (action) {
            case YES_SHOOT:
                this.shooter.setShotPosition(ShotPosition.STARTAIM);
                noShoot = false;
                break;
            case NO_SHOOT:
                this.shooter.setShotPosition(ShotPosition.STOPAIM);
                noShoot = true;
                break;
            case SHOOTGENERAL:
                this.shooter.setShotPosition(ShotPosition.GENERAL);
                if(shooter.readyToShoot()){
                    if (!elevator.ballPrimed()){
                        elevator.setAction(ElevatorAction.INDEX);
                    }
                    else{
                        elevator.setAction(ElevatorAction.PRIME);
                    }
                }else{
                    elevator.setAction(ElevatorAction.INDEX);
                }
                break;
            case SHOOTFENDER:
                this.shooter.setShotPosition(ShotPosition.FENDER);
                if(shooter.readyToShoot()){
                    if (!elevator.ballPrimed()){
                        elevator.setAction(ElevatorAction.INDEX);
                    }else{
                        elevator.setAction(ElevatorAction.PRIME);
                    }
                }else{
                    elevator.setAction(ElevatorAction.INDEX);
                }
                break;
            case NONE:
                this.intake.setAction(IntakeAction.STOP);
                this.elevator.setAction(ElevatorAction.STOP);
                this.shooter.setShotPosition(ShotPosition.NONE);
                checkBall = false;
                break;
            case INDEX:
                this.elevator.setAction(ElevatorAction.INDEX);
                this.intake.setAction(IntakeAction.IN);
                checkBall = true;
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

        if(noShoot){
            blinkenController.set(-0.89);
        }else if(checkBall){
            if(ElevatorImpl.getBall()){
                blinkenController.set(.83);
            }else{
                blinkenController.set(.61);
            }
        }else{
            if(PIDShooterTrackingImpl.canSeeTarget() == 1.0){
                blinkenController.set(0.77);
            }else{
                blinkenController.set(0.61);
            }
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
