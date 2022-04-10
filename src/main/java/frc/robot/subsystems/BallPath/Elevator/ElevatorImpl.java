package frc.robot.subsystems.BallPath.Elevator;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BallPath.Shooter.Shooter;

public class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {

    private static final double MOTOR_SPEED = 1;
    private static final double INDEX_MOTOR_SPEED = 0.35; // 0.35

    private final WPI_TalonSRX elevator;
    private final Shooter shooter;
    private static DigitalInput beam;

    private volatile ElevatorAction action = ElevatorAction.NONE;
    private volatile boolean override = false;
    private final ColorSensorV3 elevatorColorSensor;
    // boolean ballPresent = false;
    static int detectedColorElevator = 0;
    long count = 0;

    public ElevatorImpl(WPI_TalonSRX elevator, Shooter shooter, ColorSensorV3 elevatorColorSensor, DigitalInput beam) {
        super(10, TimeUnit.MILLISECONDS);
        this.elevatorColorSensor = elevatorColorSensor;
        this.elevator = elevator;
        this.shooter = shooter;
        ElevatorImpl.beam = beam;
    }

    @Override
    public void defineResources() {
        require(elevator);
        require(beam);
        require(shooter);
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
        Color detectedColor;

        // double sensorReading = this.sensor.getRangeInches();
        // // SmartDashboard.putNumber("Elevator Ultrasonic", sensorReading);
        // this.sensorSamples.add(sensorReading);
        // if (sensorSamples.size() > SAMPLE_COUNT) {
        //     this.sensorSamples.remove();
        // }
        // double meanReading = 0;
        // for (Double d : sensorSamples) {
        //     meanReading += d / sensorSamples.size();
        // }
        

        switch (this.action) {
            case STOP:
                this.elevator.set(0);
                break;
            case PRIME:
                
                // checking for ball
                // detectedColor = elevatorColorSensor.getColor();
                // System.out.println(detectedColor.red);
                // // System.out.println("Elevator: Priming");
                // if(detectedColor.red > 0.31){
                //     detectedColorElevator = 1;
                //     // System.out.println("RED");
                // }else if(detectedColor.blue > 0.31){
                //     detectedColorElevator = 2;
                //     // System.out.println("BLUE");
                // }else{
                //     detectedColorElevator = 0;
                // }

                // if (detectedColorElevator == 1 || detectedColorElevator == 2){
                //     ballPresent = true;
                //     this.elevator.set(MOTOR_SPEED);
                // }else{
                //     ballPresent = false;
                //     this.elevator.set(0);
                // }

                if (!beam.get()){
                    this.elevator.set(MOTOR_SPEED);
                }else{
                    this.elevator.set(0);
                }

                break;
            case IN:
                if (override) {
                    this.elevator.set(MOTOR_SPEED);
                }
                break;
            case OUT:
                this.elevator.set(-MOTOR_SPEED);
                
                break;
            case RUN:
                this.elevator.set(MOTOR_SPEED);
                
                break;
            case INDEX:
                if (beam.get()){
                    this.elevator.set(INDEX_MOTOR_SPEED);
                    // if(detectedColor.red > 0.31){
                    //     detectedColorElevator = 1;
                    // }else if(detectedColor.blue > 0.31){
                    //     detectedColorElevator = 2;
                    // }else{
                    //     detectedColorElevator = 0;
                    // }
                    // if (detectedColorElevator == 1 || detectedColorElevator == 2){
                    //     this.elevator.set(0);
                    //     ballPresent = true;
                    //     break;
                    // }else{
                    //     this.elevator.set(INDEX_MOTOR_SPEED);
                    //     ballPresent = false;
                    //     break;
                    // }
                } else {
                    this.elevator.set(0);
                }
                    // System.out.println("Elevator: Indexing, Last had a ball, not checking");
                break;
            case NONE:
            default:
                elevator.stopMotor();
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
