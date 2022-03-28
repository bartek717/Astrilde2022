
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

    private double leftLimit = -120.0;
    private double rightLimit = 120.0;
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
    private final double leftLimitLimelight = -0.3;
    private final double rightLimitLimelight = 0.3;

    private final PIDController shooterPid;
    boolean centerUsingLimelight = false;
    boolean aim = true;
    double currentPosition, error;
    double difference;
    
    // ### TURRET ROTATION PID ###
    private SparkMaxPIDController turret_PIDController;
    public double turret_kP = 0.14; 
    public double turret_kI = 0.000002;
    public double turret_kD = 0.05;
    public double turret_kIz = 0;
    public double turret_kFF = 0;
    public double turret_kMaxOutput = 0.6;
    public double turret_kMinOutput = -0.6;
    // public double turret_kMaxOutputsearch = 0.3;
    // public double turret_kMinOutputsearch = 0.3;

    
    private SparkMaxPIDController hood_PIDController;
    public double hood_kP = 0.5; 
    public double hood_kI = 0.000010;
    public double hood_kD = 0.05;
    public double hood_kIz = 0;
    public double hood_kFF = 0;
    public double hood_kMaxOutput = 0.3;
    public double hood_kMinOutput = -0.3;

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
    double conversion = 0.8333;

    double heightDif = h2 - h1;
    int seen = 0;
    long logItter = 0;
 

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
        System.out.println("Distance" + distance);
        double returnAmount = 0;
        double[] distances = {55.0, 77, 100, 120, 145, 167, 202.95, 244.77, 305.66};
        int[] hoodValues = {  30,   75,  80,  95, 115, 115,    125,    135,    145};
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                hoodDif = hoodValues[i] - hoodValues[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * hoodDif;
                returnAmount = hoodValues[i]-amountToAdd;
                break;
            }
        }
        // SmartDashboard.getNumber("Hood Set Rotations", returnAmount);
        System.out.println(returnAmount);

        return returnAmount;
    }

    public double getSetpointWheel(Double distance){
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd, a;
        double returnAmount = 0;
        double[] distances = {44.0,    77,  113.4, 145.5, 170.8, 220.5};
        int[] wheelValues = {5_500, 6_800,  7_500, 8_400, 8_700, 11_000};
    
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
        hoodAngle = hoodEncoder.getPosition();
        turretRotation = turretEncoder.getPosition();

        SmartDashboard.putNumber("Shooter Position", shooterEncoderReadingPosition);
        SmartDashboard.putNumber("Shooter Velocity", shooterEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Position", turretEncoder.getPosition());
        SmartDashboard.putNumber("Hood Position", hoodEncoder.getPosition());

        // getting limelight values and total distance
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tv = table.getEntry("tv");
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        canSeeTarget = tv.getDouble(0.0);
        totalAngle = a1+y;
        totalAngleRadians = Math.toRadians(totalAngle);
        rs = Math.tan(totalAngleRadians);
        totalDistance = heightDif / rs;
        SmartDashboard.putNumber("Distance", totalDistance);
        

        switch (this.requestedPosition) {
            case FENDER:
                aim = false;
                setPointShooterPID = 6_000;
                setPointHood = 39;
                setPointRotation = 0;
                shoot = true;
                break;
            case GENERAL:
                aim = true;
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterPID = getSetpointWheel(totalDistance);
                shoot = true;
                break;
            default:
                setPointHood = 0;
                shoot = false;
                setPointShooterPID = 0;
                aim = true;
                break;
        }

        if (setPointHood > hoodEncoder.getPosition() - 1.5 && setPointHood < hoodEncoder.getPosition() + 1.5){
            hoodMotor.set(0);
        }
        else{
            // setting hood setpoint
            hood_PIDController.setReference(setPointHood, CANSparkMax.ControlType.kPosition);
            SmartDashboard.putNumber("Hood SetPoint", setPointHood);
        }

        
        if(aim){
            if(turretRotation > leftLimit && turretRotation < rightLimit){
                if(canSeeTarget == 1.0 && !flipRight && !flipLeft){
                    // System.out.println("can see");
                    seen = 0;
                    // System.out.println("Can see target and in limits");
                    if(x < rightLimitLimelight && x > leftLimitLimelight){
                        SmartDashboard.putNumber("Within limits", 1);
                        turretReady = true;
                        setPointRotation = turretRotation;
                    }else{
                        setPointRotation = turretRotation + x*conversion;
                        SmartDashboard.putNumber("Within limits", 0);
                        SmartDashboard.putNumber("x", x);
                        SmartDashboard.putNumber("x conversion", x*conversion);
                        SmartDashboard.putNumber("setPointRotation", setPointRotation);
                        SmartDashboard.putNumber("turretRotation", turretRotation);
                        if(setPointRotation <= leftLimit){
                            setPointRotation = rightLimit-1;
                            flipRight = true;
                        }else if(setPointRotation >= rightLimit){
                            setPointRotation = leftLimit+1;
                            flipLeft = true;
                        }
                    }
                }else if (seen > 40){

                    // System.out.println("Cannot see target");
                    // flip right
                    if(flipLeft){
                        if(canSeeTarget == 1.0){
                            if(turretRotation + x*conversion > leftLimit && turretRotation + x*conversion < rightLimit){
                                flipLeft = false;
                                setPointRotation = turretRotation + x*conversion;
                            }
                        
                        }else if (turretRotation <= (leftLimit+10)){
                            setPointRotation = rightLimit -1;
                            flipRight=true;
                            flipLeft = false;
                            
                        }
                    // flip left
                    }else if (flipRight){
                        if(canSeeTarget == 1.0){
                            if(turretRotation + x*conversion > leftLimit && turretRotation + x*conversion < rightLimit){
                                flipRight = false;
                                setPointRotation = turretRotation + x*conversion;
                            }
                        }else if (turretRotation >= rightLimit - 10){
                            setPointRotation = leftLimit+1;
                            flipLeft=true;
                            flipRight = false;
                            
                        }
                    }
                    else{
                        setPointRotation=leftLimit-1;
                        flipLeft=true;
                    }
                }
                else{
                    seen ++;
                    // System.out.println("counting");
                }
                // flip left until you see a target?
            // just get back in the limits
            }else if(turretRotation < leftLimit){
                setPointRotation = leftLimit+1;
            }else if(turretRotation < rightLimit){
                setPointRotation = rightLimit-1;
            }
        }
        // System.out.println("Set point rotation" + setPointRotation);
        // System.out.println("Current turret Rotation" + turretRotation);
        turret_PIDController.setReference(setPointRotation, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Turret SetPoint", setPointRotation);


        // shooter wheel
        if(setPointShooterPID != 0 && shoot){
            currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
            currentOutput += 0.01; // hack "feed forward"
            currentOutput = Utils.normalizePwm(currentOutput);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);


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
        boolean turretReady = false;
        boolean hoodReady = false;
        if(Math.abs(shooterEncoderReadingVelocity) > setPointShooterPID - 500 && Math.abs(shooterEncoderReadingVelocity) < shooterEncoderReadingVelocity + 500){
            shooterReady = true;
        }
        if(turretRotation > setPointRotation - 2.5 && turretRotation < setPointRotation + 2.5){
            turretReady = true;
        }
        if(hoodAngle > setPointHood - 2 && hoodAngle < setPointHood + 2){
            hoodReady = true;
        }
        System.out.println(turretReady + " " + shooterReady + " " + hoodReady);
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
