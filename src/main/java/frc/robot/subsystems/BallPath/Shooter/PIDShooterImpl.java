package frc.robot.subsystems.BallPath.Shooter;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class PIDShooterImpl extends RepeatingIndependentSubsystem implements Shooter {

    private final TalonSRX turretMotor;
    private final TalonFX shooterMotor;
    private final TalonSRX hoodMotor;

    private double kp = 0.00175;
    private double ki = 0.00002;
    private double kd = 0.00002;

    private volatile ShotPosition requestedPosition = ShotPosition.NONE;
    private double setPointHood;
    private double setPointRotation;
    private double setPointShooterPID;

    private double shooterEncoderReadingPosition;
    private double shooterEncoderReadingVelocity;
    private double turretHoodPosition;
    private double turretEncoderReadingPosition;
    private double turretHoodVelocity;
    private double turretEncoderReadingVelocity;

    private boolean turretReady = false;
    private boolean hoodReady = false;

    private double currentOutput;

    private final double hoodBuffer = 12500;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.3;
    private final double hoodSpeed = 0.5;

    private final PIDController shooterPid;
    boolean centerUsingLimelight = false;

    // LIMELIGHT STUFF
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry targetSkew = table.getEntry("ts");
    double x, y, a, totalAngle, rs, totalDistance, totalAngleRadians;

    double a1 = 35; // angle of limelight
    double a2 = y;
    double h2 = 103; // HEIGHT OF TARGET
    double h1 = 35; // HEIGHT OF LIMELIGHT

    double heightDif = h2 - h1;
    
    

    

    public PIDShooterImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
        this.shooterPid = new PIDController(kp, ki, kd);
        this.shooterPid.setTolerance(50);
    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);
    }

    @Override
    public void setShotPosition(ShotPosition shotPosition) {
        this.requestedPosition = shotPosition;
    }

    public double getSetpointHood(double distance){
        double hoodDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        // Integer Fender = 100_000;
        // Integer Tarmac = 230_000;
        // Integer CloseLaunchPad = 300_000; // pls test
        // Integer FarLaunchPad = 330_000; // pls test
        // Integer HumanPlayer = 400_000; // pls test
        double[] distances = {55.0, 153.0, 202.95, 244.77, 305.66};
        int[] hoodValues = {100_000, 230_000, 300_000, 330_000, 400_000};

        // LinkedHashMap<Double, Integer> foundValues = new LinkedHashMap<>();
        // foundValues.put(55.0, Fender);
        // foundValues.put(153.0, Tarmac);
        // foundValues.put(202.95, CloseLaunchPad);
        // foundValues.put(244.77, FarLaunchPad);
        // foundValues.put(305.66, HumanPlayer);
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                hoodDif = hoodValues[i] - hoodValues[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * hoodDif;
                returnAmount = amountToAdd + hoodValues[i-1];
                break;
            }
        }
        return returnAmount;
    }

    public double getSetpointWheel(Double distance){
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        // Integer Fender = 3_500;
        // Integer Tarmac = 5_400;
        // Integer CloseLaunchPad = 7_000; // pls test
        // Integer FarLaunchPad = 8_000; // pls test
        // Integer HumanPlayer = 10_000; // pls test
        // HashMap<Double, Integer> foundValues = new HashMap<>();
        // foundValues.put(55.0, Fender);
        // foundValues.put(153.0, Tarmac);
        // foundValues.put(202.95, CloseLaunchPad);
        // foundValues.put(244.77, FarLaunchPad);
        // foundValues.put(305.66, HumanPlayer);
        double[] distances = {55.0, 153.0, 202.95, 244.77, 305.66};
        int[] wheelValues = {3_500, 5_400, 7_000, 8_000, 10_000};
    
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                wheelDif = wheelValues[i] - wheelValues[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * wheelDif;
                returnAmount = amountToAdd + wheelValues[i-1];
                break;
            }
        }
        return returnAmount;
    }


    @Override
    public void task(){
        shooterEncoderReadingPosition = shooterMotor.getSelectedSensorPosition();
        shooterEncoderReadingVelocity = shooterMotor.getSelectedSensorVelocity();
        turretHoodPosition = hoodMotor.getSelectedSensorPosition();
        turretHoodVelocity = hoodMotor.getSelectedSensorVelocity();
        turretEncoderReadingPosition = this.turretMotor.getSelectedSensorPosition();
        turretEncoderReadingVelocity = this.turretMotor.getSelectedSensorVelocity();

        // SmartDashboard.putNumber("Shooter Encoder reading position", shooterEncoderReadingPosition);
        // SmartDashboard.putNumber("Shooter Encoder Reading Velocity", shooterEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Encoder Reading Position", turretEncoderReadingPosition);
        SmartDashboard.putNumber("Turret Encoder Reading Velocity", turretEncoderReadingVelocity);
        // SmartDashboard.putNumber("Turret Hood Encoder reading Position", turretHoodPosition);
        // SmartDashboard.putNumber("Turret Hood Encoder Reading Velocity", turretHoodVelocity);

        switch (this.requestedPosition) {
            case FENDER:
                setPointHood = 100_000;
                setPointShooterPID = 3500; 
                setPointRotation = 0;
                centerUsingLimelight = false;
                break;
            case GENERAL:
                centerUsingLimelight = true;
                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                x = tx.getDouble(0.0);
                y = ty.getDouble(0.0);
                a = ta.getDouble(0.0);
                totalAngle = a1+a2;
                totalAngleRadians = Math.toRadians(totalAngle);
                rs = Math.tan(totalAngleRadians);
                totalDistance = heightDif / rs;
                System.out.println(totalDistance);
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterPID = getSetpointWheel(totalDistance);
                break;
            case NONE:
                centerUsingLimelight = false;
                setPointShooterPID = 0;
                setPointHood = 0;
                setPointRotation = 0;
                hoodReady = false;
                turretReady = false;
                break;
            case TEST:
                centerUsingLimelight = false;
                setPointShooterPID = 0;
                setPointHood = 0;
                setPointRotation = 0;
                break;
            default:
                break;
        }

        if (setPointHood == Double.NEGATIVE_INFINITY) {
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady = false;
        } else if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
            hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
            hoodReady = false;
        }

        if(!centerUsingLimelight){
            if (setPointRotation == Double.NEGATIVE_INFINITY) {
                turretMotor.set(ControlMode.PercentOutput, 0);
                turretReady = false;
            } else if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
                turretMotor.set(ControlMode.PercentOutput, 0);
                turretReady = true;
            }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
                turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                turretReady = false;
            }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
                turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                turretReady = false;
            }
        }else{
            if(x < 1 && x > -1){
                turretMotor.set(ControlMode.PercentOutput, 0);
                turretReady = true;
            }else if(x > 1){
                turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                turretReady = false;
            }else if(x>-1){
                turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                turretReady = false;
            }
        }


        if(setPointShooterPID != 0){
            currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
            currentOutput += 0.01; // hack "feed forward"
            currentOutput = Utils.normalizePwm(currentOutput);
            // SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
            // SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
        // System.out.println("Shooter setpoint " + setPointShooterPID + ", speed " + shooterEncoderReadingVelocity + ", power " + currentOutput);
    }

    @Override
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    @Override
    public boolean blocking() {
        boolean spinningUp = setPointShooterPID > 0 && !shooterPid.atSetpoint();
        // threshold to allow for a little bit of sensor movement around 0, not requiring absolute stillness
        double threshold = 500;
        double absoluteWheelSpeed = Math.abs(shooterEncoderReadingVelocity);
        boolean spinningDown = setPointShooterPID == 0 && absoluteWheelSpeed > threshold;
        return spinningUp || spinningDown;
        // return false;
    }

    @Override
    public boolean readyToShoot(){
        return turretReady && hoodReady && shooterPid.atSetpoint();
    }

    @Override
    public int checkBalls(){
        return 0;
    }

    @Override
    public void stopMotors(){
        setShotPosition(ShotPosition.NONE);
    }

    @Override
    public void resetSensors() {
        this.hoodMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_INIT:
                this.hoodMotor.setSelectedSensorPosition(0);
            case ON_AUTO:
            case ON_TELEOP:
            case ON_TEST:
                this.start();
                break;
            case ON_DISABLED:
                this.cancel();
                break;
            case NONE:
            default:
                this.cancel();
                this.hoodMotor.setSelectedSensorPosition(0);
                break;
        }
    }
}
