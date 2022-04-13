package frc.robot.subsystems.BallPath.Shooter;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;

public class RawShooterImpl extends RepeatingPooledSubsystem implements Shooter {

    private final TalonSRX turretMotor;
    private final TalonFX shooterMotor;
    private final TalonSRX hoodMotor;

    private volatile ShotPosition requestedPosition = ShotPosition.NONE;
    private double setPointHood;
    private double setPointRotation;
    private double setPointShooter;

    private double shooterEncoderReadingPosition;
    private double shooterEncoderReadingVelocity;
    private double turretHoodPosition;
    private double turretEncoderReadingPosition;
    private double turretHoodVelocity;
    private double turretEncoderReadingVelocity;

    private boolean turretReady = false;
    private boolean hoodReady = false;

    private final double hoodBuffer = 18_000;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.15;
    private final double hoodSpeed = .75;

    public RawShooterImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
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
        // SmartDashboard.putNumber("Turret Encoder Reading Position", turretEncoderReadingPosition);
        // SmartDashboard.putNumber("Turret Encoder Reading Velocity", turretEncoderReadingVelocity);
        // SmartDashboard.putNumber("Turret Hood Encoder reading Position", turretHoodPosition);
        // SmartDashboard.putNumber("Turret Hood Encoder Reading Velocity", turretHoodVelocity);

        switch (this.requestedPosition) {
            case TARMAC:
                setPointHood = 270_000; // to be decided
                setPointShooter = 0.4; // tbd
                setPointRotation = 0; // will probably still be 0 for the auto shot
                break;
            case FENDER:
                setPointHood = 100_000;
                setPointShooter = 0.35; // tbd
                setPointRotation = 100000;
                // System.out.println("USING THE RAW SHOOTER IMPL");
                break;
            case LAUNCHPAD_CLOSE:
                setPointHood = 0; // to be decided
                setPointShooter = 0; // tbd
                setPointRotation = 0; // tbd
                break;
            case LAUNCHPAD_FAR:
                setPointHood = 300000;
                setPointShooter = 0; // tbd
                setPointRotation = 200000;
                break;
            case NONE:
                setPointShooter = 0;
                setPointHood = 0;
                setPointRotation = 0;
                hoodReady = false;
                turretReady = false;
                break;
            case TEST:
                setPointShooter = 0.2;
                setPointHood = 100_000;
                break;
            case TEST2:
                setPointRotation = 100000;
                setPointShooter = 0;
                setPointHood = 0;
                // System.out.println("Setting shot position");
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

        this.shooterMotor.set(ControlMode.PercentOutput, Utils.normalizePwm(setPointShooter));
    }

    @Override
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    @Override
    public boolean readyToShoot(){
        boolean shooterReady = false;
        if(Math.abs(shooterEncoderReadingVelocity) > setPointShooter - 300 && Math.abs(shooterEncoderReadingVelocity) < shooterEncoderReadingVelocity + 300){
            shooterReady = true;
        }
        return turretReady && hoodReady && shooterReady;
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
            case ON_AUTO:
                this.hoodMotor.setSelectedSensorPosition(0);
                this.start();
                break;
            case ON_INIT:
            case ON_TELEOP:
            case ON_TEST:
                this.start();
                break;
            case ON_DISABLED:
            case NONE:
            default:
                this.cancel();
                this.hoodMotor.setSelectedSensorPosition(0);
                break;
        }
    }

    @Override
    public int getBallsShooter() {
        // TODO Auto-generated method stub
        return 0;
    }
}
