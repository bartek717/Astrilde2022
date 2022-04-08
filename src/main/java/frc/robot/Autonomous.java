package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.RawDriveImpl;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

public class Autonomous {
    
    private Drive drivetrain;
    private BallPath ballPath;

    private double targetDistance;

    interface Waiter {
        void waitFor(long delay, TimeUnit unit) throws InterruptedException;
    }

    private final Waiter waiter;

    private final CANSparkMax leftSide;
    private final CANSparkMax rightSide;

    public Autonomous(Waiter waiter, Drive drivetrain, BallPath ballPath){
        this.waiter = waiter;
        this.drivetrain = drivetrain;
        this.leftSide = drivetrain.getLeftSide();
        this.rightSide = drivetrain.getRightSide();
        this.ballPath = ballPath;
    }

    public boolean turn(AHRS gyro, double degree) throws InterruptedException {
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

    void setOutputRange(double percent){
        ((RawDriveImpl) this.drivetrain).setOutputRange(percent);
    }

    boolean ballPresent(){
        return this.ballPath.getElevator().ballPrimed();
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
