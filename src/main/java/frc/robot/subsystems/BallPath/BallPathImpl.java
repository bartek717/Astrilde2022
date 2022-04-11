package frc.robot.subsystems.BallPath;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.BlinkinLEDController;
import ca.team3161.lib.robot.BlinkinLEDController.Pattern;
import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
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
    private BlinkinLEDController blinkenController;
    boolean checkBall = false;
    boolean noShoot = false;
    private boolean flipped = false;
    private boolean ballSent = false;
    int startBall = 1;
    int readyShootDelay = 0;

    private volatile BallAction action = BallAction.NONE;

    public BallPathImpl(Intake intake, Elevator elevator, Shooter shooter, BlinkinLEDController blinkenController) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
        this.blinkenController = blinkenController;
    }

    @Override
    public void defineResources(){
        require(intake);
        require(elevator);
        require(shooter);
    }

    @Override
    public void setAction(BallAction inputAction) {
        this.action = inputAction;
    }

    @Override
    public void task() {
        // int ballNumber = startBall + intake.getBallsIntake() - shooter.getBallsShooter();
        // SmartDashboard.putNumber("BALL NUMBER", ballNumber);


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
                    if (elevator.ballPrimed()){
                        elevator.setAction(ElevatorAction.RUN);
                    }else{
                        intake.setAction(IntakeAction.IN);
                    }
                }else{
                    elevator.setAction(ElevatorAction.INDEX);
                }
                break;
            case SHOOTFENDER:
                this.shooter.setShotPosition(ShotPosition.FENDER);
                if(shooter.readyToShoot()){
                    if (elevator.ballPrimed()){
                        elevator.setAction(ElevatorAction.RUN);
                    }else{
                        intake.setAction(IntakeAction.IN);
                    }
                }else{
                    elevator.setAction(ElevatorAction.INDEX);
                }
                
                break;
            case NONE:
                this.intake.setAction(IntakeAction.STOP);
                this.elevator.setAction(ElevatorAction.STOP);
                this.shooter.setShotPosition(ShotPosition.STARTAIM);
                checkBall = false;
                ballSent = false;
                flipped = false;
                break;
            case INDEX:
                if (this.elevator.ballPrimed()){
                    this.intake.setAction(IntakeAction.IN);
                    if(this.intake.ballPrimed()){
                        this.intake.setAction(IntakeAction.STOP);
                    }
                } else{
                    this.elevator.setAction(ElevatorAction.INDEX);
                    this.intake.setAction(IntakeAction.IN);
                }
                
                checkBall = true;
                break;
            case OUT:
                this.elevator.setAction(ElevatorAction.OUT);
                this.intake.setAction(IntakeAction.OUT);
                break;
            case SHOOT:
                if(this.shooter.readyToShoot()){
                    this.elevator.setAction(ElevatorAction.RUN);
                }
                break;
            case STOP_SHOOTING:
            this.elevator.setAction(ElevatorAction.NONE);
            case AUTO:
            case MANUAL:
                break;
            
            default:
                intake.setAction(IntakeAction.NONE);
                elevator.setAction(ElevatorAction.NONE);
                shooter.setShotPosition(ShotPosition.NONE);
                ballSent = false;
                flipped = false;
                break;
        }

        if(noShoot){
            blinkenController.setLEDPattern(Pattern.RAINBOW_GLITTER);
        }else if(checkBall){
            if(elevator.ballPrimed()){
                blinkenController.setLEDPattern(Pattern.SOLID_SKY_BLUE);
            }else{
                blinkenController.setLEDPattern(Pattern.SOLID_RED);
            }
        }else{
            if (action.equals(BallAction.SHOOTFENDER)) {
                if (shooter.readyToShoot()){
                    blinkenController.setLEDPattern(Pattern.STROBE_WHITE);
                }  else{
                    blinkenController.setLEDPattern(Pattern.SOLID_AQUA);
                }
            } else {
                if(PIDShooterTrackingImpl.canSeeTarget() == 1.0){
                    if (shooter.readyToShoot()){
                        blinkenController.setLEDPattern(Pattern.STROBE_WHITE);
                    }  else{
                        blinkenController.setLEDPattern(Pattern.SOLID_GREEN);
                    }
                }else{
                    blinkenController.setLEDPattern(Pattern.SOLID_RED);
                }
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
                this.blinkenController.start();
                break;
            case ON_DISABLED:
            case NONE:
            default:
                this.cancel();
                this.blinkenController.cancel();
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
