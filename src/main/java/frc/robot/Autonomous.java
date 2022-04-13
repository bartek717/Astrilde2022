package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.RawDriveImpl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

public class Autonomous {
    
    private Drive drivetrain;
    private BallPath ballPath;

    private double targetDistance;

    private final CANSparkMax leftSide;
    private final CANSparkMax rightSide;

    public Autonomous(Drive drivetrain, BallPath ballPath){
        this.drivetrain = drivetrain;
        this.leftSide = drivetrain.getLeftSide();
        this.rightSide = drivetrain.getRightSide();
        this.ballPath = ballPath;
    }

    public boolean turn(AHRS gyro, double degree) throws InterruptedException {
        double target = gyro.getAngle() + degree;
        double error = target - gyro.getAngle();
        double kP = 0.003;
        double tolerance = 1;

        if (degree != 0){
            while (gyro.getAngle() < target - tolerance || gyro.getAngle() > target + tolerance){
                Thread.sleep(20);
                this.leftSide.set(kP * error);
                this.rightSide.set(-kP * error);
                error = target - gyro.getAngle();
                SmartDashboard.putNumber("LEFTSIDE POS: ", this.leftSide.get());
                SmartDashboard.putNumber("RIGHTSIDE POS: ", this.rightSide.get());
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
        double revTolerance = 2;
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

    boolean setOutputRange(double percent){
        ((RawDriveImpl) this.drivetrain).setOutputRange(percent);
        return true;
    }

    boolean ballPresent(){
        return this.ballPath.getElevator().ballPrimed() && this.ballPath.getIntake().ballPrimed();
    }

    void prepareToShoot(){
        this.ballPath.setAction(BallAction.INDEX);
    }

    void stopShooting(){
        this.ballPath.setAction(BallAction.NONE);
    }

    void shootGeneral(){
        this.ballPath.setAction(BallAction.SHOOTGENERAL);
    }

    void shootFender() {
        this.ballPath.setAction(BallAction.SHOOTFENDER);
    }

    void stop(){
        this.ballPath.setAction(BallAction.NONE);
    }
    
}
