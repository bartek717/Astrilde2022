package frc.robot.subsystems.BallPath;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Elevator.Elevator.ElevatorAction;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.Intake.IntakeAction;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;

public class BallPathImpl extends RepeatingPooledSubsystem implements BallPath {

    // private final Intake intake;
    //private final Elevator elevator;
    private final Shooter shooter;

    private volatile BallAction action = BallAction.NONE;
    
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 elevatorColorSensor = new ColorSensorV3(i2cPort);
    private static final double ELEVATOR_SPEED = 0.35;
    private static final double INTAKE_SPEED = 0.7;
    private final WPI_TalonSRX elevator;
    private final WPI_TalonSRX intake;

    public BallPathImpl(WPI_TalonSRX intake, WPI_TalonSRX elevator, Shooter shooter) {
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
        //boolean intakeLoaded = this.intake.ballPrimed();
        //boolean elevatorLoaded = this.elevator.ballPrimed();

        // boolean robotEmpty = !intakeLoaded && !elevatorLoaded;
        // boolean elevatorOnly = elevatorLoaded && !intakeLoaded;
        // boolean intakeOnly = intakeLoaded && !elevatorLoaded;
        // boolean full = intakeLoaded && elevatorLoaded;
        
        int currentColor = 0;

        switch (action) {
            case NONE:
                this.intake.set(0);
                this.elevator.set(0);
                // this.shooter.setShotPosition(ShotPosition.NONE);
                break;
            case INDEX:
                //this.elevator.setAction(ElevatorAction.INDEX);
                Color detectedColor = elevatorColorSensor.getColor();
                
                SmartDashboard.putNumber("Red", detectedColor.red);
                SmartDashboard.putNumber("Green", detectedColor.green);
                SmartDashboard.putNumber("Blue", detectedColor.blue);
                // boolean stateChanged = ballPresent != lastPresent;
                if(detectedColor.red > 0.33){
                    currentColor = 1;
                    System.out.println("RED");
                }else if(detectedColor.blue > 0.32){
                    currentColor = 2;
                    System.out.println("BLUE");
                }else{
                    currentColor = 0;
                }
                if (currentColor == 1 || currentColor == 2){
                    this.elevator.set(0);
                    
                    System.out.println("################## FOUND A BALL ###################");
                }else{
                    this.elevator.set(ELEVATOR_SPEED);
                    System.out.println("Indexing the turret");
                }
                
                this.intake.set(INTAKE_SPEED);
                break;
            case OUTTAKE:
            
                this.elevator.set(-ELEVATOR_SPEED);
                this.intake.set(-INTAKE_SPEED);
                break;
<<<<<<< Updated upstream
            case NONE:
                this.intake.setAction(IntakeAction.NONE);
                this.elevator.setAction(ElevatorAction.NONE);
                this.shooter.setShotPosition(ShotPosition.NONE);
=======
            case ELEVATOR_UP:
                this.elevator.set(ELEVATOR_SPEED);
>>>>>>> Stashed changes
                break;
            case ELEVATOR_DOWN:
                this.elevator.set(-ELEVATOR_SPEED);
            case MANUAL:
                break;
            default:
                this.intake.set(0);
                this.elevator.set(0);
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

    // @Override
    // public Intake getIntake(){
    //     return this.intake;
    // }

    // @Override
    // public Elevator getElevator(){
    //     return this.elevator;
    // }

    @Override
    public Shooter getShooter(){
        return this.shooter;
    }
}
