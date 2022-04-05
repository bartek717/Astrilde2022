package frc.robot.subsystems.BallPath.Intake;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {

    private static final double MOTOR_SPEED = 0.6;
    private static final double PRIMED_DIST_THRESHOLD = 15;
    private static final int SAMPLE_COUNT = 1;

    private final WPI_TalonSRX intake;

    private volatile IntakeAction action = IntakeAction.NONE;
    private boolean lastPresent = false;
    private final Queue<Double> sensorSamples;
    private final DigitalInput intakeSensor;
    private boolean ballThrough = false;
    private boolean stateChanged = false;

    public IntakeImpl(WPI_TalonSRX intake, DigitalInput intakeSensor) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.intakeSensor = intakeSensor;
        this.sensorSamples = new ArrayDeque<>();
    }

    @Override
    public void defineResources() {
        require(intake);
        require(intakeSensor);
    }

    @Override
    public void setAction(IntakeAction inputAction) {
        this.action = inputAction;
    }

    @Override
    public boolean ballPrimed() {
        return lastPresent;
    }

    public boolean getBall(){
        return this.intakeSensor.get();
    }

    @Override
    public void task() throws Exception {
        boolean ballPresent = this.intakeSensor.get();
        // double sensorReading = this.intakeSensor.getRangeInches();
        // // SmartDashboard.putNumber("Intake ultrasonic", sensorReading);
        // this.sensorSamples.add(sensorReading);
        // if (sensorSamples.size() > SAMPLE_COUNT) {
        //     this.sensorSamples.remove();
        // }
        // double meanReading = 0;
        // for (Double d : sensorSamples) {
        //     meanReading += d / sensorSamples.size();
        // }

        // boolean ballPresent = meanReading < PRIMED_DIST_THRESHOLD;
        // boolean stateChanged = ballPresent != lastPresent;

        switch (this.action) {
            case AUTO:
                this.intake.set(MOTOR_SPEED);
                break;
            case FEED:
                // if (ballPresent) {
                //     this.intake.stopMotor();
                // } else {
                //     this.intake.set(MOTOR_SPEED);
                // }
                break;
            case PRIME:
                // if (ballPresent) {
                //     this.intake.set(MOTOR_SPEED);
                // } else {
                //     this.intake.stopMotor();
                // }
                break;
            case STOP:
                this.intake.set(0);
                break;
            case REJECT:
                // if (ballPresent) {
                //     this.intake.set(-MOTOR_SPEED);
                // } else {
                //     this.intake.stopMotor();
                // }
                break;
            case TESTIN:
                // if(!this.ballThrough){
                //     this.intake.set(MOTOR_SPEED);
                //     if(!getBall()){
                //         stateChanged = true;
                //         System.out.println("STATE CHANGED");
                //     }else{
                //         if(stateChanged){
                //             if(getBall()){
                //                 System.out.println("STATE CHANGED 2");
                //                 ballThrough = true;
                //                 stateChanged = false;
                //             }
                //         }
                //     }
                // }else{
                //     System.out.println("Checking if ball");
                //     if(!getBall()){
                //         System.out.println("SETTING INTAKE TO 0");
                //         this.intake.set(0);
                //     }else{
                //         System.out.println("SETTING INTAKE TO RUN");
                //         this.intake.set(MOTOR_SPEED);
                //     }
                // }
                if(getBall()){
                    intake.set(MOTOR_SPEED);
                }else{
                    intake.set(0);
                }
                break;
            case TESTOUT:
                ballThrough = false;
                stateChanged = false;
                this.intake.set(-MOTOR_SPEED);
                break;
            case TESTSHOOT:
                ballThrough = false;
                stateChanged = false;
                this.intake.set(MOTOR_SPEED);
                break;
            case IN:
                this.intake.set(MOTOR_SPEED);
                break;
            case OUT:
                this.intake.set(-MOTOR_SPEED);
                break;
            case NONE:
            default:
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
