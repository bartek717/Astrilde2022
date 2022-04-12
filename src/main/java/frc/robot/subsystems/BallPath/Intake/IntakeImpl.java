package frc.robot.subsystems.BallPath.Intake;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {

    private static final double MOTOR_SPEED = 0.6;

    private final WPI_TalonSRX intake;

    private volatile IntakeAction action = IntakeAction.NONE;
    private DigitalInput beam;
    
    public IntakeImpl(WPI_TalonSRX intake, DigitalInput beam) {
        super(10, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.beam = beam;
    }

    @Override
    public void defineResources() {
        require(intake);
        require(beam);
    }

    @Override
    public void setAction(IntakeAction inputAction) {
        this.action = inputAction;
    }

    @Override
    public boolean ballPrimed() {
        return !beam.get();
    }

    @Override
    public void task() throws Exception {

        switch (this.action) {
            case IN:
                this.intake.set(MOTOR_SPEED);
                break;
            case OUT:
                this.intake.set(-MOTOR_SPEED);
                break;
            case STOP:
                this.intake.set(0);
                break;
            case INDEX:
                if (this.ballPrimed()) {
                    this.intake.set(0);
                } else {
                    this.intake.set(MOTOR_SPEED);
                }
                break;
            case NONE:
            default:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
                intake.stopMotor();
                break;
        }
        SmartDashboard.putBoolean("INTAKE BEAM BREAK", beam.get());
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
