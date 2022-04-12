package frc.robot.subsystems.BallPath.Elevator;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {

    private static final double MOTOR_SPEED = 1;
    private static final double INDEX_MOTOR_SPEED = 1; // 0.35

    private final WPI_TalonSRX elevator;
    private static DigitalInput beam;

    private volatile ElevatorAction action = ElevatorAction.NONE;
    private volatile boolean override = false;
    // boolean ballPresent = false;
    static int detectedColorElevator = 0;
    long count = 0;

    public ElevatorImpl(WPI_TalonSRX elevator, DigitalInput beam) {
        super(10, TimeUnit.MILLISECONDS);
        this.elevator = elevator;
        ElevatorImpl.beam = beam;
    }

    @Override
    public void defineResources() {
        require(elevator);
        require(beam);
    }

    @Override
    public void setAction(ElevatorAction inputAction) {
        this.action = inputAction;
    }

    @Override
    public void setGateOverride(boolean override) {
        this.override = override;
    }

    @Override
    public boolean ballPrimed() {
        // System.out.println(ballPresent);
        return !beam.get();
    }    

    @Override
    public void task() throws Exception {
        

        switch (this.action) {
            case IN:
                if (override) {
                    this.elevator.set(MOTOR_SPEED);
                }
                break;
            case OUT:
                this.elevator.set(-MOTOR_SPEED);
                
                break;
            case STOP:
                this.elevator.set(0);
                break;
            case RUN:
                this.elevator.set(MOTOR_SPEED);
                
                break;
            case INDEX:
                if (this.ballPrimed()){
                    this.elevator.set(0);
                } else {
                    this.elevator.set(INDEX_MOTOR_SPEED);
                }
                    // System.out.println("Elevator: Indexing, Last had a ball, not checking");
                break;
            case NONE:
            default:
                this.elevator.stopMotor();
                break;
        }
        SmartDashboard.putBoolean("ELEVATOR BEAM  BREAK", beam.get());
        SmartDashboard.putNumber("ELEVATOR SPEED", this.elevator.get());
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
}
