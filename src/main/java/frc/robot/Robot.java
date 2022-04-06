// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.utils.controls.DeadbandJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.JoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.DpadDirection;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.BallPath.BallPathImpl;
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Elevator.Elevator.ElevatorAction;
import frc.robot.subsystems.BallPath.Elevator.ElevatorImpl;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.Intake.IntakeAction;
import frc.robot.subsystems.BallPath.Intake.IntakeImpl;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;
import frc.robot.subsystems.BallPath.Shooter.PIDShooterImpl;
import frc.robot.subsystems.BallPath.Shooter.PIDShooterTrackingImpl;
import frc.robot.subsystems.BallPath.Shooter.RawShooterImpl;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.RawDriveImpl;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TitanBot {
  private static final String k5Ball = "Five Ball Auto";
  private static final String k4Ball = "Four Ball Auto";
  private static final String k3Ball = "Three Ball Auto";
  private static final String k2Ball = "Two Ball Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Drive drive;
  private LogitechDualAction driverPad;
  private LogitechDualAction operatorPad;  
  private BallPath ballSubsystem;
  private Intake intake;
  private Elevator elevator;
  private Shooter shooter;
  private Climber climberSubsystem;
  private boolean reverseDrive = false;
  public static final boolean DEBUG = false;
  double turretEncoderReadingPosition;
  double turretEncoderReadingVelocity;
  private UsbCamera webcam;

  private Autonomous auto;
  AHRS ahrs;


  @Override
  public int getAutonomousPeriodLengthSeconds() {
    return 15;
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotSetup() {
    System.out.println("Robot setup start");
    m_chooser.setDefaultOption("Five Ball Auto", k5Ball);
    m_chooser.setDefaultOption("Four Ball Auto", k4Ball);
    m_chooser.setDefaultOption("Three Ball Auto", k3Ball);
    m_chooser.setDefaultOption("Two Ball Auto", k2Ball);

    SmartDashboard.putData("Auto choices", m_chooser);

    // DRIVETRAIN COMPONENTS
    CANSparkMax leftControllerPrimary = new CANSparkMax(RobotMap.NEO_LEFT_DRIVE_PORTS[0], MotorType.kBrushless);
    CANSparkMax leftControllerFollower = new CANSparkMax(RobotMap.NEO_LEFT_DRIVE_PORTS[1], MotorType.kBrushless);
    CANSparkMax rightControllerPrimary = new CANSparkMax(RobotMap.NEO_RIGHT_DRIVE_PORTS[0], MotorType.kBrushless);
    CANSparkMax rightControllerFollower = new CANSparkMax(RobotMap.NEO_RIGHT_DRIVE_PORTS[1], MotorType.kBrushless);
    
    leftControllerPrimary.restoreFactoryDefaults();
    leftControllerFollower.restoreFactoryDefaults();
    rightControllerPrimary.restoreFactoryDefaults();
    rightControllerFollower.restoreFactoryDefaults();
    
    leftControllerFollower.follow(leftControllerPrimary);
    rightControllerFollower.follow(rightControllerPrimary);

    leftControllerPrimary.setSmartCurrentLimit(30);
    leftControllerFollower.setSmartCurrentLimit(30);
    rightControllerPrimary.setSmartCurrentLimit(30);
    rightControllerFollower.setSmartCurrentLimit(30);
    
    leftControllerPrimary.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftControllerFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightControllerPrimary.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightControllerFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftControllerPrimary.setInverted(true);
    
    //SpeedControllerGroup leftSide = new SpeedControllerGroup(leftMotorController1, leftMotorController2);
    //SpeedControllerGroup rightSide = new SpeedControllerGroup(rightMotorController1, rightMotorController2);
    
    // rightSide.setInverted(true);
    // Encoder leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_PORTS[0], RobotMap.LEFT_ENCODER_PORTS[1], false, Encoder.EncodingType.k2X);
    // Encoder rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_PORTS[0], RobotMap.RIGHT_ENCODER_PORTS[1], false, Encoder.EncodingType.k2X);
    RelativeEncoder leftEncoderPrimary = leftControllerPrimary.getEncoder();
    RelativeEncoder rightEncoderPrimary = rightControllerPrimary.getEncoder();

    final I2C.Port i2cPort = I2C.Port.kMXP;
    final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    

    this.drive = new RawDriveImpl(leftControllerPrimary, rightControllerPrimary, leftEncoderPrimary, rightEncoderPrimary);

    // INTAKE COMPONENTS
    WPI_TalonSRX intakeMotorController = new WPI_TalonSRX(RobotMap.INTAKE_TALON_PORT);
    Ultrasonic intakeSensor = new Ultrasonic(RobotMap.INTAKE_ULTRASONIC_PORTS[0], RobotMap.INTAKE_ULTRASONIC_PORTS[1]);
    // intakeSensor.setEnabled(true);
    this.intake = new IntakeImpl(intakeMotorController, intakeSensor);

    // SHOOTER COMPONENTS
    CANSparkMax turretMotor = new CANSparkMax(RobotMap.TURRET_PORT, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(true);
    turretMotor.setSmartCurrentLimit(20);
    // TalonSRX turretMotor = new TalonSRX(RobotMap.TURRET_PORT);

    TalonFX shooterMotor = new TalonFX(RobotMap.SHOOTER_PORT);
    shooterMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 1));
    shooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 38, 45, 0.5));
    // TalonSRX hoodMotor = new TalonSRX(RobotMap.HOOD_PORT);
    CANSparkMax hoodMotor = new CANSparkMax(RobotMap.HOOD_PORT, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setSmartCurrentLimit(20);
    hoodMotor.setInverted(true);
    this.shooter = new PIDShooterTrackingImpl(turretMotor, shooterMotor, hoodMotor);

    // ELEVATOR COMPONENTS
    WPI_TalonSRX elevatorMotorController = new WPI_TalonSRX(RobotMap.ELEVATOR_TALON_PORT);
    Ultrasonic elevatorSensor = new Ultrasonic(RobotMap.ELEVATOR_ULTRASONIC_PORTS[0], RobotMap.ELEVATOR_ULTRASONIC_PORTS[1]);
    this.elevator = new ElevatorImpl(elevatorMotorController, elevatorSensor, shooter, m_colorSensor);

    // Ultrasonic.setAutomaticMode(true);

    this.ballSubsystem = new BallPathImpl(intake, elevator, shooter);

    // Driverpad impl
    this.driverPad = new LogitechDualAction(RobotMap.DRIVER_PAD_PORT);
    this.operatorPad = new LogitechDualAction(RobotMap.OPERATOR_PAD_PORT);

    //this.climberSubsystem = new ClimberImpl();

    
    ahrs = new AHRS(SPI.Port.kMXP); 

    // register lifecycle components
    registerLifecycleComponent(driverPad);
    registerLifecycleComponent(operatorPad);
    registerLifecycleComponent(drive);
    registerLifecycleComponent(intake);
    registerLifecycleComponent(elevator);
    registerLifecycleComponent(shooter);
    registerLifecycleComponent(ballSubsystem);
    System.out.println("Robot setup complete");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // double ticks = ((PIDShooterTrackingImpl) this.shooter).getHoodMotor().getEncoder().getCountsPerRevolution() * ((PIDShooterTrackingImpl) this.shooter).getHoodMotor().getEncoder().getPosition();
    // SmartDashboard.putNumber("Hooder Encoder Reading", ticks);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousSetup() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    driverPad.start();
    // operatorPad.start();
    drive.start();
    // intake.start();
    // elevator.start();
    // shooter.start();
    // ballSubsystem.start();

    auto = new Autonomous(this::waitFor, this.drive, this.ballSubsystem);
  }

  /** This function is called periodically during autonomous. 
   * @throws InterruptedException
   * */
  @Override
  public void autonomousRoutine() throws InterruptedException {
    drive.resetEncoderTicks();

    double[][] targets = {{0, 0}};

    switch (m_autoSelected) {
      case k5Ball:
        targets = new double[][] {
          {}, 
          {}, 
          {}, 
          {}
        };
        break;
      case k4Ball:
        targets = new double[][] {
          {}, 
          {},
          {}
        };
        break;
      case k3Ball:
        targets = new double[][] {
          {0, 0},
          {35, 0}, 
          {-10, 0}, 
          {60, 105}
        };
        break;
      case k2Ball:
        targets = new double[][] {
          {}
        };
        break;
      default:
        // Put default auto code here
        break;
    }

    int index = 0;
    boolean doneAuto = false;

    while (!doneAuto) {
      if (index == targets.length-1) {
        doneAuto = true;
      }
      auto.resetPosition();
      auto.setDriveDistance(targets[index][0]);
      System.out.println("Is about to turn");
      auto.turn(ahrs, targets[index][1]);
      System.out.println("Has turned");
      auto.resetPosition();
      Timer.delay(1);
      boolean doneDriving = false;
      auto.prepareToShoot();
      while (!doneDriving) {
        System.out.println("Is driving");
        doneDriving = auto.drive();
      }
      // auto.stopDriving();
      auto.shoot();
      Timer.delay(5);
      auto.stop();
      // auto.stopDriving();
      index += 1;
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopSetup() {
    System.out.println("telleop setup");
    JoystickMode deadbandMode = new DeadbandJoystickMode(0.05);
    this.driverPad.setMode(ControllerBindings.LEFT_STICK, ControllerBindings.X_AXIS, deadbandMode.andThen(x -> x * .45));
    this.driverPad.setMode(ControllerBindings.RIGHT_STICK, ControllerBindings.Y_AXIS, new InvertedJoystickMode().andThen(deadbandMode));

    this.operatorPad.bind(ControllerBindings.INTAKE, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.INDEX));
    this.operatorPad.bind(ControllerBindings.INTAKE, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));
    this.operatorPad.bind(ControllerBindings.OUTTAKE, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.OUT));
    this.operatorPad.bind(ControllerBindings.OUTTAKE, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));
    this.operatorPad.bind(ControllerBindings.OVERRIDE_ELEVATOR_GATE, this.elevator::setGateOverride);

    this.operatorPad.bind(ControllerBindings.SHOOT_FENDER, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.SHOOTFENDER));
    this.operatorPad.bind(ControllerBindings.SHOOT_FENDER, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));

    this.operatorPad.bind(ControllerBindings.SHOOT_GENERAL, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.SHOOTGENERAL));
    this.operatorPad.bind(ControllerBindings.SHOOT_GENERAL, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));


    this.ballSubsystem.setAction(BallPath.BallAction.MANUAL);
    
    driverPad.start();
    operatorPad.start();
    drive.start();
    intake.start();
    elevator.start();
    shooter.start();
    ballSubsystem.start();


  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopRoutine() {
      double forward, turn;
      forward = this.driverPad.getValue(ControllerBindings.RIGHT_STICK, ControllerBindings.Y_AXIS);
      turn = this.driverPad.getValue(ControllerBindings.LEFT_STICK, ControllerBindings.X_AXIS);

      this.drive.drive(forward, turn); 
      }

  static DpadDirection angleToDpadDirection(int angle) {
      for (DpadDirection d : DpadDirection.values()) {
          if (d.getAngle() == angle) {
              return d;
          }
      }
      return DpadDirection.NONE;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledSetup() {
    driverPad.cancel();
    operatorPad.cancel();
    drive.cancel();
    intake.cancel();
    elevator.cancel();
    shooter.cancel();
    ballSubsystem.cancel();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledRoutine() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testSetup() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testRoutine() {}

}
