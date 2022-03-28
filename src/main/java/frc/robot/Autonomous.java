package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.Drivetrain.Drive;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

// PIDCONTROLLER IMPORTS
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.BallPath.Intake.Intake;

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

    private final PIDController positionPIDController;

    private final CANSparkMax leftSide;
    private final CANSparkMax rightSide;

    public Autonomous(Waiter waiter, Drive drivetrain, BallPath ballPath){
        this.waiter = waiter;
        this.drivetrain = drivetrain;
        this.leftSide = drivetrain.getLeftSide();
        this.rightSide = drivetrain.getRightSide();
        this.ballPath = ballPath;
        this.positionPIDController = new PIDController(kP, kI, kD);
        this.positionPIDController.setTolerance(DRIVE_DIST_TOLERANCE);
    }

    public void turn(AHRS gyro, double degree) {
        gyro.reset();
        double error = degree - gyro.getAngle();
        double kP = 0.1;

        while (gyro.getAngle() < degree - 2 && gyro.getAngle() > degree + 2){
            this.leftSide.set(kP * error);
            this.rightSide.set(-kP * error);
            error = degree - gyro.getAngle();
        }

        this.leftSide.set(0);
        this.rightSide.set(0);
    }


    private double convertIR(double inches){
        double gearRatio = 5;
        double wheelCircumference = 2 * Math.PI * 2;

        double revs = (inches / wheelCircumference) * gearRatio;

        return revs;
    }

    public void setDriveDistance(double distance) {
        this.targetDistance = convertIR(distance);
        positionPIDController.setSetpoint(this.targetDistance);
    }

    public boolean drive(){
        /*
        :targetPosition: distance to be driven in inches -> double
        */

        double dampener = 0.3;

        double position = (this.leftSide.getEncoder().getPosition() + this.rightSide.getEncoder().getPosition()) / 2;

        double nextSpeed = positionPIDController.calculate(position);

        if (!positionPIDController.atSetpoint()) {
            this.drivetrain.drive(nextSpeed * dampener, 0);
        }
        return positionPIDController.atSetpoint();
    }

    void prepareToShoot(){
        // boolean intakeLoaded = ballPath.getIntake().ballPrimed();
        // boolean elevatorLoaded = ballPath.getElevator().ballPrimed();
        // boolean robotFull = intakeLoaded && elevatorLoaded;

        ballPath.setAction(BallPath.BallAction.SHOOTGENERAL);
        ballPath.getIntake().setAction(Intake.IntakeAction.IN);
    }

    void shoot() throws InterruptedException {
        ballPath.getElevator().setAction(Elevator.ElevatorAction.AUTO);
    }

    void stopDriving() {
        drivetrain.drive(0, 0);
    }

    void stop(){
        drivetrain.drive(0, 0);
        this.ballPath.setAction(BallAction.NONE);
    }
    
}
