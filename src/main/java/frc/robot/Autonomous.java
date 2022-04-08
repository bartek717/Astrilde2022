package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.RawDriveImpl;

import java.util.concurrent.TimeUnit;

import javax.sound.sampled.ReverbType;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

// PIDCONTROLLER IMPORTS
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.Intake.IntakeAction;

public class Autonomous {

    private static final double DRIVE_DIST_TOLERANCE = 3; // TODO determine what this should really be
    
    private Drive drivetrain;
    private BallPath ballPath;

    private double kP = 0.01;
    private double kI = 0.0025;
    private double kD = 0.005;

    private double targetDistance;

    interface Waiter {
        void waitFor(long delay, TimeUnit unit) throws InterruptedException;
    }

    private final Waiter waiter;

    // private final PIDController positionPIDController;

    private final CANSparkMax leftSide;
    private final CANSparkMax rightSide;

    private final SparkMaxPIDController leftPIDController;
    private final SparkMaxPIDController rightPIDController;

    public Autonomous(Waiter waiter, Drive drivetrain, BallPath ballPath){
        this.waiter = waiter;
        this.drivetrain = drivetrain;
        this.leftSide = drivetrain.getLeftSide();
        this.rightSide = drivetrain.getRightSide();
        this.leftPIDController = ((RawDriveImpl) drivetrain).getLeftPIDController();
        this.rightPIDController = ((RawDriveImpl) drivetrain).getLeftPIDController();
        this.ballPath = ballPath;
        // this.positionPIDController = new PIDController(kP, kI, kD);
        // this.positionPIDController.setTolerance(DRIVE_DIST_TOLERANCE);
    }

    public boolean turn(AHRS gyro, double degree) throws InterruptedException {
        // gyro.reset();
        double target = gyro.getAngle() + degree;
        double error = target - gyro.getAngle();
        double kP = 0.005;
        double tolerance = 2;

        if (degree != 0){
            while (gyro.getAngle() < target - tolerance || gyro.getAngle() > target + tolerance){
                Thread.sleep(20);
                this.leftSide.set(kP * error);
                this.rightSide.set(-kP * error);
                error = target - gyro.getAngle();
                if (Robot.DEBUG){
                    System.out.println(error);
                }
            }
        }

        this.leftSide.set(0);
        this.rightSide.set(0);

        return true;
    }

    public void resetPosition(){
        // positionPIDController.reset();
        // this.leftSide.getEncoder().setPosition(0);
        // this.rightSide.getEncoder().setPosition(0);
        this.drivetrain.resetEncoderTicks();
    }


    private double convertIR(double inches){
        double gearRatio = 5;
        double wheelCircumference = 2 * Math.PI * 2;

        double revs = (inches / wheelCircumference) * gearRatio;

        return revs;
    }

    public boolean setDriveDistance(double distance) {
        this.targetDistance = convertIR(distance);
        return true;
        // positionPIDController.setSetpoint(this.targetDistance);
    }

    public void drive(){
        /*
        :targetPosition: distance to be driven in inches -> double
        */
        ((RawDriveImpl) this.drivetrain).setPosition(this.targetDistance);

    }

    public boolean atPosition(){
        boolean arrived = false;
        double revTolerance = 0.5;
        double leftSidePos = this.leftSide.getEncoder().getPosition();
        double rightSidePos = this.rightSide.getEncoder().getPosition();

        if (Robot.DEBUG){
            System.out.println("LeftSidePos: " + leftSidePos + ", RightSidePos: " + rightSidePos + " Target Pos: " + this.targetDistance);
        }
        boolean leftSideArrvied = leftSidePos > this.targetDistance - revTolerance && leftSidePos < this.targetDistance + revTolerance;
        boolean rightSideArrvied = rightSidePos > this.targetDistance - revTolerance && rightSidePos < this.targetDistance + revTolerance;
        
        if (leftSideArrvied && rightSideArrvied) {
            arrived = true;
        }
        
        return arrived;
    }

    void prepareToShoot(){
        
        ballPath.setAction(BallAction.INDEX);
    }

    void stopShooting(){
        this.ballPath.setAction(BallAction.YES_SHOOT);
    }

    boolean shoot() {
        this.ballPath.setAction(BallAction.SHOOTGENERAL);
        return true;
    }

    void stop(){
        this.ballPath.setAction(BallAction.NONE);
    }
    
}
