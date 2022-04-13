
package frc.robot.subsystems.BallPath.Shooter;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class PIDShooterTrackingImpl extends RepeatingIndependentSubsystem implements Shooter {
    
    private final CANSparkMax turretMotor;
    private final TalonFX shooterMotor;
    // private final TalonSRX hoodMotor;
    private final CANSparkMax hoodMotor;
    private final CANSparkMax hoodShooterMotor;


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
    private RelativeEncoder hoodEncoder;
    private RelativeEncoder turretEncoder;
    private RelativeEncoder hoodShooterMotorEncoder;

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

    private boolean shootFender = false;

    private final PIDController shooterPid;
    boolean centerUsingLimelight = false;
    boolean aim = true;
    double currentPosition, error;
    double difference;
    int count;

    // turret hood motor pif
    private SparkMaxPIDController hoodShooterMotor_PIDController;
    public double hoodShooterMotor_kP = 0.00015; 
    public double hoodShooterMotor_kI = 0.00000;
    public double hoodShooterMotor_kD = 0.01;
    public double hoodShooterMotor_kIz = 0;
    public double hoodShooterMotor_kFF = 0.00009;
    public double hoodShooterMotor_kMaxOutput = 1;
    public double hoodShooterMotor_kMinOutput = -1;

    // ### TURRET ROTATION PID ###
    private SparkMaxPIDController turret_PIDController;
    public double turret_kP = 0.14; 
    public double turret_kI = 0.000002;
    public double turret_kD = 0.07;
    public double turret_kIz = 0;
    public double turret_kFF = 0;
    public double turret_kMaxOutput = 0.7;
    public double turret_kMinOutput = -0.7;
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
    double x, y, a;
    static double canSeeTarget;
    double totalAngle;
    double rs;
    double totalDistance;
    double totalAngleRadians;

    double a1 = 35; // angle of limelight
    double a2 = y;
    // System.out.println(y);
    double h2 = 103; // HEIGHT OF TARGET
    double h1 = 35; // HEIGHT OF LIMELIGHT
    double conversion = 0.8333;

    double heightDif = h2 - h1;
    int seen = 0;
    long logItter = 0;
    private double hoodShooterMotorSpeed = 0;
    int ballsOut = 0;
    boolean flipped = false;
    boolean ramped = false;
 
    

    public PIDShooterTrackingImpl(CANSparkMax turretMotor, TalonFX shooterMotor, CANSparkMax hoodMotor, CANSparkMax hoodShooterMotor) {
        super(10, TimeUnit.MILLISECONDS);

        this.hoodShooterMotor = hoodShooterMotor;
        this.hoodShooterMotorEncoder = hoodShooterMotor.getEncoder();
        this.hoodShooterMotor_PIDController = hoodShooterMotor.getPIDController();
        hoodShooterMotor_PIDController.setP(hoodShooterMotor_kP);
        hoodShooterMotor_PIDController.setI(hoodShooterMotor_kI);
        hoodShooterMotor_PIDController.setD(hoodShooterMotor_kD);
        hoodShooterMotor_PIDController.setIZone(hoodShooterMotor_kIz);
        hoodShooterMotor_PIDController.setFF(hoodShooterMotor_kFF);
        hoodShooterMotor_PIDController.setOutputRange(hoodShooterMotor_kMinOutput, hoodShooterMotor_kMaxOutput);

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
        this.shooterPid.setTolerance(2000); // FIXME tune better so tolerance is tighter
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
    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);
        require(hoodShooterMotor);
    }

    @Override
    public void setShotPosition(ShotPosition shotPosition) {
        this.requestedPosition = shotPosition;
    }

    public double getSetpointHood(double distance){
        double hoodDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        // System.out.println("Distance" + distance);
        double returnAmount = 0;
        double[] distances = {55.0, 77, 100, 120, 145, 167, 202.95, 244.77, 305.66};
        int[] hoodValues = {  5,   40,  60,  90, 110, 115,    125,    135,    135};
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
        // System.out.println(returnAmount);

        return returnAmount;
    }

    
    public double getSetpointHoodShooter(Double distance){
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = {44.0,    77,  113.4, 145.5, 170.8, 220.5};
        double[] wheelValues = {2000, 7000,  8000, 9000, 10000, 10500};
    
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

    public double getSetpointWheel(Double distance){
        double wheelDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = {44.0,    77,  113.4, 145.5, 170.8, 220.5};
        int[] wheelValues = {5_500, 5_700,  5_900, 6_100, 8_500, 9_700};
    
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
        SmartDashboard.putNumber("RETURN AMOUNT", returnAmount); 
        return returnAmount;
    }

    public static double canSeeTarget(){
        return canSeeTarget;
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
                shootFender = true;
                aim = false;
                setPointShooterPID = 7000; //4700
                setPointHood = 0;
                setPointRotation = 0;
                shoot = true;
                hoodShooterMotorSpeed = 9000; // 4250
                // System.out.println("FENDER");
                break;
            case GENERAL:
                shootFender = false;
                aim = true;
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterPID = getSetpointWheel(totalDistance);
                shoot = true;
                hoodShooterMotorSpeed = getSetpointHoodShooter(totalDistance);
                // System.out.println("GENERAL");
                break;
            case STOPAIM:
                shootFender = false;
                aim = false;
                setPointShooterPID = 0;
                setPointHood = 0;
                setPointRotation = 0;
                shoot = false;
                hoodShooterMotorSpeed = 0;
                // System.out.println("STOPAIM");
                break;
            case STARTAIM:
                // System.out.println("STARTAIM");
            default:
                // System.out.println("DEFAULT");
                shootFender = false;
                setPointHood = 0;
                shoot = false;
                setPointShooterPID = 0;
                hoodShooterMotorSpeed = 0;
                aim = true;
                
                break;
        }

        this.hoodShooterMotor_PIDController.setReference(hoodShooterMotorSpeed, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Hood SHOOTER CURRENT VELOCITY", hoodShooterMotorEncoder.getVelocity());
        SmartDashboard.putNumber("Hood SHOOTER CURRRENT SETPOINT", hoodShooterMotorSpeed);

        // hood position
        if (setPointHood > hoodEncoder.getPosition() - 1.5 && setPointHood < hoodEncoder.getPosition() + 1.5){
            hoodMotor.set(0);
        }
        else{
            // setting hood setpoint
            hood_PIDController.setReference(setPointHood, CANSparkMax.ControlType.kPosition);
            SmartDashboard.putNumber("Hood SetPoint", setPointHood);
        }

        // turret position
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
                }else if (seen > 20){

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
                        if(turretRotation > 0){
                            setPointRotation=leftLimit+1;
                            flipLeft=true;
                        }else{
                            setPointRotation=rightLimit-1;
                            flipRight = true;
                        }
                        
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
            currentOutput += setPointShooterPID == 0 ? 0 : 0.05; // hack "feed forward"
            currentOutput = Utils.normalizePwm(currentOutput);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);

        // hood shooter wheel


    }    
    

    @Override
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    @Override
    public boolean readyToShoot(){
        boolean shooterReady = shooterPid.atSetpoint();
        boolean turretReady = false;
        boolean hoodReady = false;
        boolean hoodShooterReady = false;
        // if(shooterReady){
        //     System.out.println(shooterEncoderReadingVelocity);
        // }
        if(turretRotation > setPointRotation - 2.5 && turretRotation < setPointRotation + 2.5){
            turretReady = true;
        }
        if(hoodAngle > setPointHood - 1 && hoodAngle < setPointHood + 1){
            hoodReady = true;
        }
        if(hoodShooterMotorEncoder.getVelocity() > hoodShooterMotorSpeed - 500 && hoodShooterMotorEncoder.getVelocity() < hoodShooterMotorSpeed + 500){
            hoodShooterReady = true;

        }
        // System.out.println(hoodShooterMotorEncoder.getVelocity());
        // System.out.println(hoodAngle);
        // System.out.println(setPointHood);
        System.out.println(turretReady + " " + shooterReady + " " + hoodReady + " " + hoodShooterReady);
        // if(shooterReady && !ramped && !flipped){
        //     ramped = true;
        //     flipped = false;
        //     System.out.println(ramped + " " + flipped);
        // }else{
        //     if(!flipped && ramped){
        //         ballsOut += 1;
        //         flipped = true;
        //         ramped = false;
        //     }
        // }
        return turretReady && shooterReady && hoodReady && hoodShooterReady;
        
    }

    public int getBallsShooter(){
        return ballsOut;
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
                this.turretEncoder.setPosition(0);
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
