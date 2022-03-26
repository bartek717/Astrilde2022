
package frc.robot.subsystems.BallPath.Shooter;

import java.nio.file.spi.FileSystemProvider;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.FloatArraySerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;

public class PIDShooterTrackingImpl extends RepeatingIndependentSubsystem implements Shooter {
    
    private final CANSparkMax turretMotor;
    private final TalonFX shooterMotor;
    // private final TalonSRX hoodMotor;
    private final CANSparkMax hoodMotor;


    private double kp = PIDShooterImpl.kp; // 0.00175
    private double ki = PIDShooterImpl.ki; // 0.00002
    private double kd = PIDShooterImpl.kd; // 0.00002

    private double leftLimit = -90.0;
    private double rightLimit = 90.0;
    private double degreesToTicks = 5555; //  find actual values

    private volatile ShotPosition requestedPosition = ShotPosition.NONE;
    private double setPointHood;
    private double setPointShooterPID;
    private double setPointRotation;

    private double shooterEncoderReadingPosition;
    private double shooterEncoderReadingVelocity;
    private double hoodAngle;
    private double turretRotation;
    private double turretHoodVelocity;
    private double turretEncoderReadingVelocity;
    private Spark blinkinController;
    private RelativeEncoder hoodEncoder;
    private RelativeEncoder turretEncoder;

    private boolean turretReady = false;
    private boolean hoodReady = false;

    private boolean shoot = false;
    private double currentOutput;
    private boolean flipRight = false;
    private boolean flipLeft = false;

    private final double hoodBuffer = 12500;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.05;
    private final double hoodSpeed = 0.5;
    private final double leftLimitLimelight = -1;
    private final double rightLimitLimelight = 1;

    private final PIDController shooterPid;
    boolean centerUsingLimelight = false;
    boolean aim = true;
    double currentPosition, error;
    double difference;
    
    // ### TURRET ROTATION PID ###
    private SparkMaxPIDController turret_PIDController;
    public double turret_kP = 0.5; 
    public double turret_kI = 0.000010;
    public double turret_kD = 0.05;
    public double turret_kIz = 0;
    public double turret_kFF = 0;
    public double turret_kMaxOutput = 1;
    public double turret_kMinOutput = -1;

    
    private SparkMaxPIDController hood_PIDController;
    public double hood_kP = 0.5; 
    public double hood_kI = 0.000010;
    public double hood_kD = 0.05;
    public double hood_kIz = 0;
    public double hood_kFF = 0;
    public double hood_kMaxOutput = 1;
    public double hood_kMinOutput = -1;

    // LIMELIGHT STUFF
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry targetSkew = table.getEntry("ts");
    double x, y, a, canSeeTarget, totalAngle, rs, totalDistance, totalAngleRadians;

    double a1 = 35; // angle of limelight
    double a2 = y;
    // System.out.println(y);
    double h2 = 103; // HEIGHT OF TARGET
    double h1 = 35; // HEIGHT OF LIMELIGHT

    double heightDif = h2 - h1;
 

    public PIDShooterTrackingImpl(CANSparkMax turretMotor, TalonFX shooterMotor, CANSparkMax hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        // Turret Rotation
        this.turretMotor = turretMotor;
        this.turretEncoder = turretMotor.getEncoder();
        this.difference = turretEncoder.getCountsPerRevolution();
        this.turret_PIDController = turretMotor.getPIDController();
        turret_PIDController.setP(turret_kP);
        turret_PIDController.setI(turret_kI);
        turret_PIDController.setD(turret_kD);
        turret_PIDController.setIZone(turret_kIz);
        turret_PIDController.setFF(turret_kFF);
        turret_PIDController.setOutputRange(turret_kMinOutput, turret_kMaxOutput);

        SmartDashboard.putNumber("Turret P Gain", turret_kP);
        SmartDashboard.putNumber("Turret I Gain", turret_kI);
        SmartDashboard.putNumber("Turret D Gain", turret_kD);
        SmartDashboard.putNumber("Turret I Zone", turret_kIz);
        SmartDashboard.putNumber("Turret Feed Forward", turret_kFF);
        SmartDashboard.putNumber("Turret Max Output", turret_kMaxOutput);
        SmartDashboard.putNumber("Turret Min Output", turret_kMinOutput);
        SmartDashboard.putNumber("Turret Set Rotations", 0);
        // Shooter Wheel
        this.shooterMotor = shooterMotor;
        this.shooterPid = new PIDController(kp, ki, kd);
        SmartDashboard.putNumber("Shooter Set Speed", 0);
        // Hood
        this.hoodMotor = hoodMotor;
        this.hoodEncoder = hoodMotor.getEncoder();
        this.hood_PIDController = hoodMotor.getPIDController();
        hood_PIDController.setP(hood_kP);
        hood_PIDController.setI(hood_kI);
        hood_PIDController.setD(hood_kD);
        hood_PIDController.setIZone(hood_kIz);
        hood_PIDController.setFF(hood_kFF);
        hood_PIDController.setOutputRange(hood_kMinOutput, hood_kMaxOutput);

        SmartDashboard.putNumber("Hood P Gain", hood_kP);
        SmartDashboard.putNumber("Hood I Gain", hood_kI);
        SmartDashboard.putNumber("Hood D Gain", hood_kD);
        SmartDashboard.putNumber("Hood I Zone", hood_kIz);
        SmartDashboard.putNumber("Hood Feed Forward", hood_kFF);
        SmartDashboard.putNumber("Hood Max Output", hood_kMaxOutput);
        SmartDashboard.putNumber("Hood Min Output", hood_kMinOutput);
        SmartDashboard.putNumber("Hood Set Rotations", 0);

        // LEDs
        this.blinkinController = new Spark(7);
       
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
        double[] distances = {55.0, 153.0, 202.95, 244.77, 305.66};
        int[] hoodValues = {10, 20, 30, 40, 50};
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
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd, a;
        double returnAmount = 0;
        double[] distances = {44.0, 113.4, 145.5, 170.8, 220.5};
        int[] wheelValues = {5_500, 7_500, 9_500, 10_000, 11_000};
    
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                wheelDif = wheelValues[i] - wheelValues[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * wheelDif;
                returnAmount = wheelValues[i] - amountToAdd;
                break;
            }
        }
        return returnAmount;
    }

    
    public CANSparkMax getHoodMotor(){
        return this.hoodMotor;
    }

    @Override
    public void task(){
        // getting and posting encoder reading positions for turret, hood and shooter
        shooterEncoderReadingPosition = shooterMotor.getSelectedSensorPosition();
        shooterEncoderReadingVelocity = shooterMotor.getSelectedSensorVelocity();
        // turretHoodPosition = hoodMotor.getSelectedSensorPosition();

        hoodAngle = hoodEncoder.getPosition();
        // turretHoodVelocity = hoodMotor.getSelectedSensorVelocity();

        turretRotation = turretEncoder.getPosition();

        SmartDashboard.putNumber("Shooter Position", shooterEncoderReadingPosition);
        SmartDashboard.putNumber("Shooter Velocity", shooterEncoderReadingVelocity);
        //SmartDashboard.putNumber("Turret Position", turretEncoderReadingPosition);
        //SmartDashboard.putNumber("Hood Position", turretHoodPosition);

        
        

        double tP = SmartDashboard.getNumber("Turret P Gain", 0);
        double tI = SmartDashboard.getNumber("Turret I Gain", 0);
        double tD = SmartDashboard.getNumber("Turret D Gain", 0);
        double tIz = SmartDashboard.getNumber("Turret I Zone", 0);
        double tFF = SmartDashboard.getNumber("Turret Feed Forward", 0);
        double tMax = SmartDashboard.getNumber("Turret Max Output", 0);
        double tMin = SmartDashboard.getNumber("Turret Min Output", 0);
        // double tRotations = SmartDashboard.getNumber("Turret Set Rotations", 0);

        if((tP != turret_kP)) { turret_PIDController.setP(tP); turret_kP = tP; }
        if((tI != turret_kI)) { turret_PIDController.setI(tI); turret_kI = tI; }
        if((tD != turret_kD)) { turret_PIDController.setD(tD); turret_kD = tD; }
        if((tIz != turret_kIz)) { turret_PIDController.setIZone(tIz); turret_kIz = tIz; }
        if((tFF != turret_kFF)) { turret_PIDController.setFF(tFF); turret_kFF = tFF; }
        if((tMax != turret_kMaxOutput) || (tMin != turret_kMinOutput)) { 
            turret_PIDController.setOutputRange(tMin, tMax); 
            turret_kMinOutput = tMin; turret_kMaxOutput = tMax; 
        }

        // turret_PIDController.setReference(tRotations, CANSparkMax.ControlType.kPosition);

        // SmartDashboard.putNumber("Turret SetPoint", tRotations);
        SmartDashboard.putNumber("Turret Position", turretEncoder.getPosition());

        double hP = SmartDashboard.getNumber("Hood P Gain", 0);
        double hI = SmartDashboard.getNumber("Hood I Gain", 0);
        double hD = SmartDashboard.getNumber("Hood D Gain", 0);
        double hIz = SmartDashboard.getNumber("Hood I Zone", 0);
        double hFF = SmartDashboard.getNumber("Hood Feed Forward", 0);
        double hMax = SmartDashboard.getNumber("Hood Max Output", 0);
        double hMin = SmartDashboard.getNumber("Hood Min Output", 0);
        // double hRotations = SmartDashboard.getNumber("Hood Set Rotations", 0);

        if((hP != hood_kP)) { hood_PIDController.setP(hP); hood_kP = hP; }
        if((hI != hood_kI)) { hood_PIDController.setI(hI); hood_kI = hI; }
        if((hD != hood_kD)) { hood_PIDController.setD(hD); hood_kD = hD; }
        if((hIz != hood_kIz)) { hood_PIDController.setIZone(hIz); hood_kIz = hIz; }
        if((hFF != hood_kFF)) { hood_PIDController.setFF(hFF); hood_kFF = hFF; }
        if((hMax != hood_kMaxOutput) || (hMin != hood_kMinOutput)) { 
            hood_PIDController.setOutputRange(hMin, hMax); 
            hood_kMinOutput = hMin; hood_kMaxOutput = hMax; 
        }

        // hood_PIDController.setReference(hRotations, CANSparkMax.ControlType.kPosition);

        // SmartDashboard.putNumber("Hood SetPoint", hRotations);
        SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());
        

        switch (this.requestedPosition) {
            case FENDER:
                aim = false;
                setPointShooterPID = 5_500;
                setPointHood = 30;
                setPointRotation = 0;
                break;
            case GENERAL:
                aim = true;
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterPID = getSetpointWheel(totalDistance);
                shoot = true;
                break;
            default:
                setPointHood = 30;
                shoot = false;
                setPointShooterPID = 0;
                setPointRotation = 0;
                aim = true;
                break;
        }

        hood_PIDController.setReference(setPointHood, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Hood SetPoint", setPointHood);
        turret_PIDController.setReference(setPointRotation, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Turret SetPoint", setPointRotation);

        




        if(aim){
            System.out.println("aiming");
            // only get the distance if we are aiming
            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            ta = table.getEntry("ta");
            tv = table.getEntry("tv");
            x = tx.getDouble(0.0);
            y = ty.getDouble(0.0);
            a = ta.getDouble(0.0);
            canSeeTarget = tv.getDouble(0.0);
            totalAngle = a1+y;
            totalAngleRadians = Math.toRadians(totalAngle);
            rs = Math.tan(totalAngleRadians);
            totalDistance = heightDif / rs;
            SmartDashboard.putNumber("Distance", totalDistance);
            if(turretRotation > leftLimit && turretRotation < rightLimit && !flipLeft && !flipRight){
                // System.out.println("Within encoder limits");
                if(canSeeTarget==1.0){
                    System.out.println("Can see target");
                    // if we are in the encoder limits, can see the target, and do not want to "flip"
                    if(x < rightLimitLimelight && x > leftLimitLimelight){
                        turretReady = true;
                        //error = 0;
                    }
                    else{
                        turretReady = false;
                    }
                    setPointRotation = turretRotation + (x*.3333);
                    System.out.println(x + " " + x*.333333 + " " + setPointRotation);
                    if (!(turretRotation >= setPointRotation - 1 && turretRotation <= setPointRotation + 1)){
                        turret_PIDController.setReference(setPointRotation, CANSparkMax.ControlType.kPosition);
                        SmartDashboard.putNumber("Turret SetPoint", setPointRotation);
                    }
                }
                // else{
                //     // if we cannot see the target, flip to one of the limits
                //     if(turretRotation >= 0){
                //         flipRight = true;
                //         turretReady = false;
                //     }else if(turretRotation < 0){
                //         flipLeft = true;
                //         turretReady = false;
                //     }
                // }
            }
        }    
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
        boolean shooterReady = false;
        if(Math.abs(shooterEncoderReadingVelocity) > setPointShooterPID - 500 && Math.abs(shooterEncoderReadingVelocity) < shooterEncoderReadingVelocity + 500){
            shooterReady = true;
        }
        return turretReady && shooterReady && hoodReady;
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
        
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_INIT:
                // this.hoodMotor.setSelectedSensorPosition(0);
                this.hoodEncoder.setPosition(0);
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
                this.hoodEncoder.setPosition(0);
                break;
        }
    }
}
