package frc.robot.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;

public class RawDriveImpl extends RepeatingPooledSubsystem implements Drive {

    // motor controller groups
    private CANSparkMax leftSide;
    private CANSparkMax rightSide;
    
    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController rightPIDController;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    Pose2d pose;

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final double alP = 0.108;
    private final double alI = 0;
    private final double alD = 0.08;

    private final double arP = 0.16;
    private final double arI = 0;
    private final double arD = 0.08;

    public RawDriveImpl(CANSparkMax leftSide, CANSparkMax rightSide) {        super(20, TimeUnit.MILLISECONDS);
        // basic drivetrain stuff
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.leftEncoder = leftSide.getEncoder();
        this.rightEncoder = rightSide.getEncoder(); 

        // Left PID controller setup
        this.leftPIDController = this.leftSide.getPIDController();
        this.leftPIDController.setP(alP);
        this.leftPIDController.setI(alI);
        this.leftPIDController.setD(alD);

        // Right PID controller setup
        this.rightPIDController = this.rightSide.getPIDController();
        this.rightPIDController.setP(arP);
        this.rightPIDController.setI(arI);
        this.rightPIDController.setD(arD);
    }

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.66));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());


    @Override
    public void defineResources() {
        require(leftSide);
        require(rightSide);
        
        require(leftEncoder);
        require(rightEncoder);
    }

    public double lSpeed(){
      return leftSide.getEncoder().getVelocity()/5*2*Math.PI*Units.inchesToMeters(2)/60;
    }

    public double rSpeed(){
      return rightSide.getEncoder().getVelocity()/5*2*Math.PI*Units.inchesToMeters(2)/60;
    }

    @Override
    public void task() throws Exception {
        pose = odometry.update(getHeading(), lSpeed(), rSpeed());
    }

    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(-this.gyro.getAngle());
    }

    public static WheelSpeeds arcadeDriveIK(double xSpeed, double zRotation, boolean squareInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    
        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        // if (squareInputs) {
        //   xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        //   zRotation = Math.copySign(zRotation * zRotation, zRotation);
        // }
    
        double leftSpeed;
        double rightSpeed;
    
        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    
        if (xSpeed >= 0.0) {
          // First quadrant, else second quadrant
          if (zRotation >= 0.0) {
            leftSpeed = maxInput;
            rightSpeed = xSpeed - zRotation;
          } else {
            leftSpeed = xSpeed + zRotation;
            rightSpeed = maxInput;
          }
        } else {
          // Third quadrant, else fourth quadrant
          if (zRotation >= 0.0) {
            leftSpeed = xSpeed + zRotation;
            rightSpeed = maxInput;
          } else {
            leftSpeed = maxInput;
            rightSpeed = xSpeed - zRotation;
          }
        }
    
        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }
    
        return new WheelSpeeds(leftSpeed, rightSpeed);
      }

    public void drive(double leftSpeed, double rotation){
        var speeds = arcadeDriveIK(leftSpeed, rotation, true);

        leftSide.set(speeds.left);
        rightSide.set(speeds.right);
    }

    @Override
    public Pair<Double, Double> getEncoderTicks(){
      return Pair.of(this.leftEncoder.getPosition(), this.rightEncoder.getPosition());
    }


    @Override
    public void resetEncoderTicks() {
        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);
    }

    public void setTurnAngle(double angle){
      
    }

    public void setPosition(double pos) {
      this.leftPIDController.setReference(pos, ControlType.kPosition);
      // System.out.println("leftPID Val: " + this.leftEncoder.getPosition());
      this.rightPIDController.setReference(pos, ControlType.kPosition);
      // System.out.println("rightPID Val: " + this.rightEncoder.getPosition());
    }

    public SparkMaxPIDController getLeftPIDController(){
      return this.leftPIDController;
    }

    public SparkMaxPIDController getRightPIDController(){
      return this.rightPIDController;
    }

    public void setOutputRange(double percent){
      this.leftPIDController.setOutputRange(-percent, percent);
      this.rightPIDController.setOutputRange(-percent, percent);
    }

    @Override
    public CANSparkMax getLeftSide(){
      return this.leftSide;
    }

    @Override
    public CANSparkMax getRightSide(){
      return this.rightSide;
    }
    
    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
      switch (current) {
        case ON_INIT:
        case ON_AUTO:
        case ON_TELEOP:
        case ON_TEST:
          this.start();
          break;
        case ON_DISABLED:
        case NONE:
        default:
          this.leftEncoder.setPosition(0);
          this.rightEncoder.setPosition(0);
          this.cancel();
          break;
    }
  }

}