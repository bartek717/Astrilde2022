
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

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.PIDDriveImpl;
import frc.robot.subsystems.Drivetrain.RawDriveImpl;


import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;

public class PIDShooterTrackingImpl extends RepeatingIndependentSubsystem implements Shooter {
    
    private final TalonSRX turretMotor;
    private final TalonFX shooterMotor;
    private final TalonSRX hoodMotor;
    private final Drive drive;

    private double kp = PIDShooterImpl.kp; // 0.00175
    private double ki = PIDShooterImpl.ki; // 0.00002
    private double kd = PIDShooterImpl.kd; // 0.00002

    private double leftLimit = 200000.0;
    private double rightLimit = 200000.0;

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

    private boolean shoot = false;
    private double currentOutput;
    private boolean flipRight = false;
    private boolean flipLeft = false;

    private final double hoodBuffer = 12500;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.2;
    private final double hoodSpeed = 0.5;
    private final double leftLimitLimelight = -1;
    private final double rightLimitLimelight = 1;
    private final double lights = 0.0;
    private final double turretTicksToDegree = 2888; //based off of memory change when given the oppertunity

    private final PIDController shooterPid;
    boolean centerUsingLimelight = false;
    boolean aim = false;

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
 

    public PIDShooterTrackingImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor, Drive drive) {
        super(10, TimeUnit.MILLISECONDS);
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
        this.shooterPid = new PIDController(kp, ki, kd);
        this.drive = drive;
       
        SmartDashboard.putNumber("Shooter Set Speed", 0);
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

    public double getTimeToTarget(double distance){
        double hoodDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double time = 0;
        double[] distances = {55.0, 153.0, 202.95, 244.77, 305.66};
        double[] times = {1, 1.2, 1.8, 2.2, 2.5};
        for (int i = 1; i < distances.length; i++) {
            double key = distances[i];
            if(distance < key){
                distDif = distances[i] - distances[i-1];
                hoodDif = times[i] - times[i-1];
                difFromUpper = distances[i] - distance;
                percentToAdd = difFromUpper / distDif;
                amountToAdd = percentToAdd * hoodDif;
                time = amountToAdd + times[i-1];
                break;
            }
        }
        return time;
    }

    public double getSetpointHood(double distance){
        double hoodDif, distDif, difFromUpper, percentToAdd, amountToAdd;
        double returnAmount = 0;
        double[] distances = {55.0, 153.0, 202.95, 244.77, 305.66};
        int[] hoodValues = {100_000, 230_000, 300_000, 330_000, 400_000};
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
    // can probably end up making all of these into one call if we want to


    @Override
    public void task(){
        // getting and posting encoder reading positions for turret, hood and shooter
        shooterEncoderReadingPosition = shooterMotor.getSelectedSensorPosition();
        shooterEncoderReadingVelocity = shooterMotor.getSelectedSensorVelocity();
        turretHoodPosition = hoodMotor.getSelectedSensorPosition();
        turretHoodVelocity = hoodMotor.getSelectedSensorVelocity();
        turretEncoderReadingPosition = this.turretMotor.getSelectedSensorPosition();
        turretEncoderReadingVelocity = this.turretMotor.getSelectedSensorVelocity();

        SmartDashboard.putNumber("Shooter Encoder reading position", shooterEncoderReadingPosition);
        SmartDashboard.putNumber("Shooter Encoder Reading Velocity", shooterEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Encoder Reading Position", turretEncoderReadingPosition);
        SmartDashboard.putNumber("Turret Encoder Reading Velocity", turretEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Hood Encoder reading Position", turretHoodPosition);
        SmartDashboard.putNumber("Turret Hood Encoder Reading Velocity", turretHoodVelocity);

        switch (this.requestedPosition) {
            case FENDER:
                aim = false;
                setPointShooterPID = 5_500;
                setPointHood = 100_000;
                break;
            case GENERAL:
                aim = true;
                shoot = true;
                break;
            case NONE:
                shoot = false;
                setPointShooterPID = 2000;
                aim = true;
                System.out.println("AIM TRUE");
            case TEST:
            default:
                shoot = false;
                setPointShooterPID = 2000;
                aim = true;
                break;
        }
        // turret movement
        // if the user does not want to do a fender shot
        if(aim){
            double currentLeftVelocity = drive.getLeftVelocity(); 
            double currentRightVelocity = drive.getRightVelocity();
            double velocityTrajectory = Math.max(currentLeftVelocity, currentRightVelocity); // I think this is right?
            System.out.println("AIMING");
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
            // if we are shooting from a standstill
            if(velocityTrajectory < 0.1){
                setPointHood = getSetpointHood(totalDistance);
                setPointShooterPID = getSetpointWheel(totalDistance);
                // no need to adjust turret on shots with no movement
                if(turretEncoderReadingPosition > leftLimit && turretEncoderReadingPosition < rightLimit && !flipLeft && !flipRight){
                    if(canSeeTarget==1.0){
                        // if we are in the encoder limits, can see the target, and do not want to "flip"
                        if(x < rightLimitLimelight && x > leftLimitLimelight){
                            turretMotor.set(ControlMode.PercentOutput, 0);
                            turretReady = true;
                        }else if(x > rightLimitLimelight){
                            turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                            turretReady = false;
                        }else if(x<leftLimitLimelight){
                            turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                            turretReady = false;
                        }
                    }else{
                        // if we cannot see the target, flip to one of the limits
                        if(turretEncoderReadingPosition >= 0){
                            flipLeft = true;
                            turretReady = false;
                        }else if(turretEncoderReadingPosition < 0){
                            flipRight = true;
                            turretReady = false;
                        }
                    }
                // if we are past the left limit, flip to the right
                }else if(turretEncoderReadingPosition < leftLimit || flipRight == true){
                    flipRight = true;
                    turretReady = false;
                    if(turretEncoderReadingPosition < rightLimit - turretBuffer && canSeeTarget == 0.0){
                        turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                    }else{
                        flipRight = false;
                    }
                // if we are at the right limit, flip to the left
                }else if(turretEncoderReadingPosition > rightLimit || flipLeft == true){
                    turretReady = false;
                    flipLeft = true;
                    if(turretEncoderReadingPosition > leftLimit + turretBuffer && canSeeTarget == 0.0){
                        turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                    }else{
                        flipLeft = false;
                    }
                }  
            // otherwise we are moving at velocity  
            }else{
                // restating so its easier for me, will remove later
                double turretDegree = turretEncoderReadingPosition / turretTicksToDegree;
                double currentDistance = totalDistance;
                double velocityTrajectoryMoving = velocityTrajectory;
                double timeToTarget = getTimeToTarget(currentDistance); // this should be with the actual shot vector but I dont know how that would work
                double displacement = velocityTrajectoryMoving * timeToTarget;
                double shootingDistance = Math.sqrt(Math.pow(currentDistance, 2)+Math.pow(displacement, 2)-(2*currentDistance*displacement*Math.cos(Math.toRadians(turretDegree))));
                double shootingAngle = Math.asin(Math.toRadians(currentDistance*Math.sin(Math.toRadians(turretDegree))/shootingDistance));
                if (shootingAngle > 50){
                    shootingAngle = 50;
                }
                setPointHood = getSetpointHood(shootingDistance);
                setPointShooterPID = getSetpointWheel(shootingDistance);
                if(turretEncoderReadingPosition > leftLimit && turretEncoderReadingPosition < rightLimit && !flipLeft && !flipRight){
                    if(canSeeTarget==1.0){
                        // if we are in the encoder limits, can see the target, and do not want to "flip"
                        if(x < rightLimitLimelight + shootingAngle && x > leftLimitLimelight + shootingAngle){
                            turretMotor.set(ControlMode.PercentOutput, 0);
                            turretReady = true;
                        }else if(x > rightLimitLimelight + shootingAngle){
                            turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                            turretReady = false;
                        }else if(x<leftLimitLimelight + shootingAngle){
                            turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                            turretReady = false;
                        }
                    }else{
                        // if we cannot see the target, flip to one of the limits
                        if(turretEncoderReadingPosition >= 0){
                            flipLeft = true;
                            turretReady = false;
                        }else if(turretEncoderReadingPosition < 0){
                            flipRight = true;
                            turretReady = false;
                        }
                    }
                // if we are past the left limit, flip to the right
                }else if(turretEncoderReadingPosition < leftLimit || flipRight == true){
                    flipRight = true;
                    turretReady = false;
                    if(turretEncoderReadingPosition < rightLimit - turretBuffer && canSeeTarget == 0.0){
                        turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                    }else{
                        flipRight = false;
                    }
                // if we are at the right limit, flip to the left
                }else if(turretEncoderReadingPosition > rightLimit || flipLeft == true){
                    turretReady = false;
                    flipLeft = true;
                    if(turretEncoderReadingPosition > leftLimit + turretBuffer && canSeeTarget == 0.0){
                        turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                    }else{
                        flipLeft = false;
                    }

                }
            }   
        // if we want to shoot the fender shot, set the turret position to be 0 (straight)
        // this may change if we have a turret that can turn 180 degrees
        }else{
            if(turretEncoderReadingPosition >= 0 - turretBuffer && turretEncoderReadingPosition <= 0 + turretBuffer){
                turretMotor.set(ControlMode.PercentOutput, 0);
                turretReady = true;
            }else if(turretEncoderReadingPosition <= 0 - turretBuffer){
                turretMotor.set(ControlMode.PercentOutput, turretSpeed);
                turretReady = false;
            }else if (turretEncoderReadingPosition >= 0 + turretBuffer){
                turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
                turretReady  = false;
            }
        }
        
        // setpoint for the shooter wheel

        if(setPointShooterPID != 0){
            currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
            currentOutput += 0.01; // hack "feed forward"
            currentOutput = Utils.normalizePwm(currentOutput);
        } else {
            currentOutput = 0;
        }
        this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);

        // hood setpoint
        if(shoot){
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
        }else{
            hoodReady = false;
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
