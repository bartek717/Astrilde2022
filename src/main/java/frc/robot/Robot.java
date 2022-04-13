// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import ca.team3161.lib.robot.BlinkinLEDController;
import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.utils.controls.CubedJoystickMode;
import ca.team3161.lib.utils.controls.DeadbandJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.JoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction.DpadDirection;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.BallPathImpl;
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Elevator.Elevator.ElevatorAction;
import frc.robot.subsystems.BallPath.Elevator.ElevatorImpl;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Intake.Intake.IntakeAction;
import frc.robot.subsystems.BallPath.Intake.IntakeImpl;
import frc.robot.subsystems.BallPath.Shooter.BangBangShooterTrackingImpl;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberImpl;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.RawDriveImpl;


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
  private XBoxOneController driverPad;
  private XBoxOneController operatorPad;  
  private BallPath ballSubsystem;
  private Intake intake;
  private Elevator elevator;
  private Shooter shooter;
  private Climber climberSubsystem;
  public static final boolean DEBUG = false;
  double turretEncoderReadingPosition;
  double turretEncoderReadingVelocity;
  private boolean set = false;
  private Autonomous auto;
  AHRS ahrs;
  public static boolean toggle;


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
    
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);

    m_chooser.setDefaultOption("Two Ball Auto", k2Ball);
    m_chooser.addOption("Three Ball Auto", k3Ball);
    m_chooser.addOption("Four Ball Auto", k4Ball);
    m_chooser.addOption("Five Ball Auto", k5Ball);

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

    DigitalInput elevatorBeam = new DigitalInput(0);
    DigitalInput intakeBeam = new DigitalInput(2);

    this.drive = new RawDriveImpl(leftControllerPrimary, rightControllerPrimary);

    // INTAKE COMPONENTS
    WPI_TalonSRX intakeMotorController = new WPI_TalonSRX(RobotMap.INTAKE_TALON_PORT);
    this.intake = new IntakeImpl(intakeMotorController, intakeBeam);

    // SHOOTER COMPONENTS
    CANSparkMax turretMotor = new CANSparkMax(RobotMap.TURRET_PORT, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(true);
    turretMotor.setSmartCurrentLimit(20);
    // TalonSRX turretMotor = new TalonSRX(RobotMap.TURRET_PORT);

    TalonFX shooterMotor = new TalonFX(RobotMap.SHOOTER_PORT);
    shooterMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 1));
    shooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 38, 45, 0.5));
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    // TalonSRX hoodMotor = new TalonSRX(RobotMap.HOOD_PORT);
    CANSparkMax hoodMotor = new CANSparkMax(RobotMap.HOOD_PORT, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setSmartCurrentLimit(20);
    hoodMotor.setInverted(true);

    BlinkinLEDController blinkenController = new BlinkinLEDController(8);

    CANSparkMax hoodShooterMotor = new CANSparkMax(RobotMap.HOOD_SHOOTER_PORT, MotorType.kBrushless);
    hoodShooterMotor.restoreFactoryDefaults();
    hoodShooterMotor.setSmartCurrentLimit(20);
    hoodShooterMotor.setInverted(true);
    hoodShooterMotor.setIdleMode(IdleMode.kCoast);

    // this.shooter = new PIDShooterTrackingImpl(turretMotor, shooterMotor, hoodMotor, hoodShooterMotor);
    this.shooter = new BangBangShooterTrackingImpl(turretMotor, shooterMotor, hoodMotor, hoodShooterMotor);

    // ELEVATOR COMPONENTS
    WPI_TalonSRX elevatorMotorController = new WPI_TalonSRX(RobotMap.ELEVATOR_TALON_PORT);
    this.elevator = new ElevatorImpl(elevatorMotorController, elevatorBeam);

    this.ballSubsystem = new BallPathImpl(intake, elevator, shooter,blinkenController);
    

    // Driverpad impl
    this.driverPad = new XBoxOneController(RobotMap.DRIVER_PAD_PORT);
    this.operatorPad = new XBoxOneController(RobotMap.OPERATOR_PAD_PORT);

    // CLIMBER COMPONENTS.
    WPI_TalonSRX primaryClimberMotorController = new WPI_TalonSRX(RobotMap.CLIMBER_TALON_PORTS[0]); 
    WPI_TalonSRX followerClimberMotorController = new WPI_TalonSRX(RobotMap.CLIMBER_TALON_PORTS[1]); 
    CANSparkMax shoulderMotorController = new CANSparkMax(RobotMap.NEO_SHOULDER_LIFTER_PORT, MotorType.kBrushless); 

    // Restore factory defaults method (of which is not supported [under the same name] for Talon SRX)?
    followerClimberMotorController.follow(primaryClimberMotorController);
    followerClimberMotorController.setInverted(InvertType.OpposeMaster);  // Invert the follower?

    this.climberSubsystem = new ClimberImpl(primaryClimberMotorController, followerClimberMotorController, shoulderMotorController);
    // System.out.println("CLIMBER" + this.climberSubsystem);
    this.climberSubsystem.resetClimberPosition();
    ahrs = new AHRS(SPI.Port.kMXP); 

    // register lifecycle components
    registerLifecycleComponent(driverPad);
    registerLifecycleComponent(operatorPad);
    registerLifecycleComponent(drive);
    registerLifecycleComponent(intake);
    registerLifecycleComponent(elevator);
    registerLifecycleComponent(shooter);
    registerLifecycleComponent(ballSubsystem);
    registerLifecycleComponent(climberSubsystem);
    // System.out.println("Robot setup complete");
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
    operatorPad.start();
    drive.start();
    intake.start();
    elevator.start();
    shooter.start();
    ballSubsystem.start();

    auto = new Autonomous(this.drive, this.ballSubsystem);
  }

  /** This function is called periodically during autonomous. 
   * @throws InterruptedException
   * */
  @Override
  public void autonomousRoutine() throws InterruptedException {

    double[][] targets; // {distance, angle, shotCheck, speedLimitCheck, speedLimit}
    //{38,0, 0}, {89,112, 1}, {155,-29, 1}, {-50,0, 0}, {0, 0, 1}

    switch (m_autoSelected) {
      case k5Ball:
        targets = new double[][] {
          {38, 0, 0, 0},
          {89, 112, 1, 1}, 
          {155, -29, 1, 1}, 
          {-50, 0, 0, 0},
          {0, 0, 1, 1}
        };
        break;
      case k4Ball:
        targets = new double[][] {
          {38, 0, 0, 0},
          {89, 112, 1, 1}, 
          {155, -29, 1, 1}, 
          {-50, 0, 0, 0},
          {0, 0, 1, 1}
        };
        break;
      case k3Ball:
        targets = new double[][] {
          {38, 0, 0, 0},
          {89, 112, 1, 1}, 
          {0, 0, 1, 1}
        };
        break;
      case k2Ball:
        targets = new double[][] {
          {13, 0, 0, 0},
          {25, 0, 1, 1}
          // {0, 45, 0, 0}
        };
        break;
      default:
        targets = new double[][] {{0, 0, 0, 0}};
        break;
    }

    int index = 0;
    boolean doneAuto = false;

    auto.resetPosition();
    auto.setOutputRange(0.7);

    while (!doneAuto) {
      // System.out.println("INDEX: " + index);
      auto.prepareToShoot();
  
      if (targets[index][1] != 0) { // turn cycle not complete
        auto.turn(ahrs, targets[index][1]);
        auto.resetPosition();
      }
      
      auto.setDriveDistance(targets[index][0]);
      

      while (!auto.atPosition()){ // drive cycle not complete 
        auto.drive();
        if (index == 0){
          Timer.delay(2);
        }
      }

      auto.resetPosition();
      auto.setDriveDistance(0);
      auto.drive();

      if (targets[index][2] != 0){ // shoot cycle not complete
        if (targets[index][3] == 0){
          auto.shootFender();
        } else {
          auto.shootGeneral();
        }
        Timer.delay(5);
      }

      index += 1;
      
      if (index == targets.length) { // If auto is complete
        doneAuto = true;
      }
    }
    auto.stop();
    auto.setOutputRange(1);
  }

  public boolean getToggle() {
    return toggle;
  }

  public void setToggle(boolean t) {
      toggle = t;
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopSetup() {
    System.out.println("telleop setup");
    JoystickMode deadbandMode = new DeadbandJoystickMode(0.05);
    this.driverPad.setMode(ControllerBindings.LEFT_STICK, ControllerBindings.X_AXIS, deadbandMode.andThen(x -> x * 1).andThen(new SquaredJoystickMode()));
    this.driverPad.setMode(ControllerBindings.RIGHT_STICK, ControllerBindings.Y_AXIS, new InvertedJoystickMode().andThen(deadbandMode));

    this.operatorPad.bind(ControllerBindings.OVERRIDE_ELEVATOR_GATE, this.elevator::setGateOverride);

    this.operatorPad.bind(ControllerBindings.SHOOT_FENDER, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.SHOOTFENDER));
    this.operatorPad.bind(ControllerBindings.SHOOT_FENDER, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));

    this.operatorPad.bind(ControllerBindings.SHOOT_GENERAL, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.SHOOTGENERAL));
    this.operatorPad.bind(ControllerBindings.SHOOT_GENERAL, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));

    this.operatorPad.bind(ControllerBindings.NOT_AIM, PressType.PRESS, () -> setToggle(!getToggle()));
    // this.operatorPad.bind(ControllerBindings.ELEVATOR_IN, PressType.PRESS, ()-> this.ballSubsystem.setAction(BallAction.SHOOT));
    // this.operatorPad.bind(ControllerBindings.ELEVATOR_IN, PressType.RELEASE, ()-> this.ballSubsystem.setAction(BallAction.STOP_SHOOTING));

    // this.operatorPad.bind(ControllerBindings.INTAKE, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.INDEX));
    // this.operatorPad.bind(ControllerBindings.INTAKE, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));
    
    // this.operatorPad.bind(ControllerBindings.OUTAKE, PressType.PRESS, () -> this.ballSubsystem.setAction(BallAction.OUT));
    // this.operatorPad.bind(ControllerBindings.OUTAKE, PressType.RELEASE, () -> this.ballSubsystem.setAction(BallAction.NONE));

    this.operatorPad.bind(ControllerBindings.DEPLOY_CLIMBER, PressType.PRESS, ()-> this.climberSubsystem.primeClimber());
    this.operatorPad.bind(ControllerBindings.DEPLOY_CLIMBER, PressType.RELEASE, () -> this.climberSubsystem.none());

    this.ballSubsystem.setAction(BallPath.BallAction.MANUAL);

    // Testing climber bindings.
    this.operatorPad.setMode(ControllerBindings.LEFT_STICK, ControllerBindings.Y_AXIS, new InvertedJoystickMode().andThen(new CubedJoystickMode()));
    this.operatorPad.setMode(ControllerBindings.RIGHT_STICK, ControllerBindings.Y_AXIS, new InvertedJoystickMode().andThen(new CubedJoystickMode()));
    
    driverPad.start();
    operatorPad.start();
    drive.start();
    intake.start();
    elevator.start();
    shooter.start();
    ballSubsystem.start();
    climberSubsystem.start();  // Hopefully this is correct?
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopRoutine() {
      // Drivetrain.
      double forward, turn;
      forward = this.driverPad.getValue(ControllerBindings.RIGHT_STICK, ControllerBindings.Y_AXIS);
      turn = this.driverPad.getValue(ControllerBindings.LEFT_STICK, ControllerBindings.X_AXIS);

      this.drive.drive(forward, turn); 

      double intake, outake;

      intake = this.operatorPad.getValue(ControllerBindings.INTAKE, ControllerBindings.LEFT_TRIGGER_AXIS);
      outake = this.operatorPad.getValue(ControllerBindings.OUTAKE, ControllerBindings.RIGHT_TRIGGER_AXIS);

      if (intake > 0.9 && !this.ballSubsystem.getAction().equals(BallAction.OUT)){
        this.ballSubsystem.setAction(BallAction.INDEX);
      } else if (this.ballSubsystem.getAction().equals(BallAction.INDEX)){
        this.ballSubsystem.setAction(BallAction.NONE);
      }

      if (outake > 0.9 && !this.ballSubsystem.getAction().equals(BallAction.INDEX)){
        this.ballSubsystem.setAction(BallAction.OUT);
      } else if (this.ballSubsystem.getAction().equals(BallAction.OUT)){
        this.ballSubsystem.setAction(BallAction.NONE);
      }

      // Ballpath.
      if(toggle){
        this.ballSubsystem.setAction(BallAction.NO_SHOOT);
        set = false;
      }else{
        if(!set){
          this.ballSubsystem.setAction(BallAction.YES_SHOOT);
          set = true;
        }
      }

      // Climber (attempt).
      double climber, shoulderSpeed;
      climber = this.operatorPad.getValue(ControllerBindings.LEFT_STICK, ControllerBindings.Y_AXIS);
      shoulderSpeed = this.operatorPad.getValue(ControllerBindings.RIGHT_STICK, ControllerBindings.Y_AXIS);

      if (Robot.DEBUG) {
        System.out.println("PLEASE SEE THIS: " + climber + " " + shoulderSpeed);
      }

      this.climberSubsystem.extendElbow(climber);
      this.climberSubsystem.extendShoulder(shoulderSpeed);
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
    climberSubsystem.cancel();

    ballSubsystem.setAction(BallAction.NONE);
    shooter.setShotPosition(ShotPosition.NONE);
    elevator.setAction(ElevatorAction.NONE);
    intake.setAction(IntakeAction.NONE);
    climberSubsystem.resetClimberPosition();
    setToggle(false);
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
