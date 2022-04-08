package frc.robot.subsystems.BallPath.Intake;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {

    private static final double MOTOR_SPEED = 0.7;
    private static final double PRIMED_DIST_THRESHOLD = 15;
    private static final int SAMPLE_COUNT = 11;

    private final WPI_TalonSRX intake;

    private volatile IntakeAction action = IntakeAction.NONE;
    private boolean lastPresent = false;
    private boolean[] sensorSamples;
    private DigitalInput beam;

    private boolean sent = false;
    private boolean flipped = false;
    private int ballNumber = 0;
    List<Boolean> list=new ArrayList<Boolean>(Arrays.asList(new Boolean[4]));
    

    public IntakeImpl(WPI_TalonSRX intake, DigitalInput beam) {
        super(20, TimeUnit.MILLISECONDS);
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
        return lastPresent;
    }

    public int getBallsIntake(){
        return ballNumber;
    }

    @Override
    public void task() throws Exception {
        double trueball = 0;
        double falseball = 0;
        boolean sensorReading = this.beam.get();
        this.list.add(sensorReading);
        if (list.size() > SAMPLE_COUNT) {
            this.list.remove(0);
        }
        for (boolean d : list) {
            if(d){
                trueball += 1;
            }else{
                falseball += 1;
            }
        }

        boolean noBallPresent = trueball > falseball;
        // boolean stateChanged = ballPresent != lastPresent;

        switch (this.action) {
            case AUTO:
                this.intake.set(MOTOR_SPEED);
                break;
            case FEED:
                if (!noBallPresent) {
                    this.intake.stopMotor();
                } else {
                    this.intake.set(MOTOR_SPEED);
                }
                break;
            case PRIME:
                if (!noBallPresent) {
                    this.intake.set(MOTOR_SPEED);
                } else {
                    this.intake.stopMotor();
                }
                break;
            case STOP:
                this.intake.set(0);
                break;
            case REJECT:
                if (!noBallPresent) {
                    this.intake.set(-MOTOR_SPEED);
                } else {
                    this.intake.stopMotor();
                }
                break;
            case TEST:
                this.intake.set(MOTOR_SPEED);
                break;
            case IN:
                this.intake.set(MOTOR_SPEED);
                if(noBallPresent){
                    flipped = false;
                }else{
                    if(!flipped){
                        ballNumber += 1;
                        flipped = true;
                    }
                }
                break;
            case OUT:
                this.intake.set(-MOTOR_SPEED);
                if(noBallPresent){
                    flipped = false;
                }else{
                    if(!flipped){
                        ballNumber -= 1;
                        flipped = true;
                    }
                }
                break;
            case NONE:
            default:
                flipped = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
                intake.stopMotor();
                break;
        }

        // lastPresent = ballPresent;
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
