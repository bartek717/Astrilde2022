package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.Drivetrain.Drive;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.Pair;

// PIDCONTROLLER IMPORTS
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import frc.robot.subsystems.BallPath.Intake.Intake;

public class Autonomous {

    private static final double DRIVE_DIST_TOLERANCE = 3; // TODO determine what this should really be
    
    private Drive drivetrain;
    private BallPath ballPath;
    private double wheelCircumference = Math.pow((Math.PI*2), 2);

    private double kP = 0.01;
    private double kI = 0.0025;
    private double kD = 0.005;

    private double zRotation = 0;
    private double targetDistance;

    interface Waiter {
        void waitFor(long delay, TimeUnit unit) throws InterruptedException;
    }

    private final Waiter waiter;

    private final PIDController positionPIDController;

    public Autonomous(Waiter waiter, Drive drivetrain, BallPath ballPath){
        this.waiter = waiter;
        this.drivetrain = drivetrain;
        this.ballPath = ballPath;
        this.positionPIDController = new PIDController(kP, kI, kD);
        this.positionPIDController.setTolerance(DRIVE_DIST_TOLERANCE);
    }

    double calcTicks(double encoderTicks){
        // Calculates ticks per revolution(shaft rotation)
        double gearRatio = 8;

        double ticks = encoderTicks / gearRatio * wheelCircumference; // ticksPer cancels so it's just encoderTicks / gearRatio

        return ticks;
    }

    // double convertIT(double distance){
    //     // Convert Inches to Ticks
    //     double gearRatio = 8;
    //     return (distance / wheelCircumference / gearRatio) * 128;
    // }

    // double convertTI(double ticks){
    //     // Convert Ticks to Inches
    //     return ticks * wheelCircumference;
    // }

    double positionPIDCalc(Double encoderTicks){
        // PID for both sides

        return positionPIDController.calculate(encoderTicks);
    }

    public void setDriveDistance(double distance) {
        this.targetDistance = distance;
        positionPIDController.setSetpoint(this.targetDistance);
    }

    public boolean drive(){
        /*
        :targetPosition: distance to be driven in inches -> double
        */

        Pair<Double, Double> encoderTicks = this.drivetrain.getEncoderTicks();

        double averageEncoderTicks = (encoderTicks.getFirst() + encoderTicks.getSecond()) / 2;

        double nextSpeed = positionPIDCalc(averageEncoderTicks);

        drivetrain.drive(nextSpeed * 0.875 /* fudge it slower */, zRotation);

        return positionPIDController.atSetpoint();
    }

    void prepareToShoot(){
        // boolean intakeLoaded = ballPath.getIntake().ballPrimed();
        // boolean elevatorLoaded = ballPath.getElevator().ballPrimed();
        // boolean robotFull = intakeLoaded && elevatorLoaded;

        ballPath.setAction(BallPath.BallAction.AUTO);
        //ballPath.getIntake().setAction(Intake.IntakeAction.AUTO);
        ballPath.getShooter().setShotPosition(Shooter.ShotPosition.TARMAC);
    }

    void shoot() throws InterruptedException {
        //ballPath.getElevator().setAction(Elevator.ElevatorAction.AUTO);
    }

    void stopDriving() {
        drivetrain.drive(0, 0);
    }

    void stop(){
        this.ballPath.setAction(BallAction.NONE);
        this.ballPath.getShooter().setShotPosition(ShotPosition.NONE);
        drivetrain.drive(0, 0);
    }
    
}
