package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;

public final class RobotMap {
    // controller ports
    public static final int DRIVER_PAD_PORT = 0;
    public static final int OPERATOR_PAD_PORT = 1;

    // ENCODERS PORTS (change when ports are final)
    // public static final int[] LEFT_ENCODER_PORTS = {0, 0}; 
    // public static final int[] RIGHT_ENCODER_PORTS = {0, 0}; 

    // assumes current motor config, can be changed later (change when ports are final)

    public static final int[] NEO_LEFT_DRIVE_PORTS = {1, 2};
    public static final int[] NEO_RIGHT_DRIVE_PORTS = {3, 4};

    public static final Port LEFT_COLOR_SENSOR_PORT = Port.kOnboard;
    public static final Port RIGHT_COLOR_SENSOR_PORT = Port.kOnboard;

    public static final int[] INTAKE_ULTRASONIC_PORTS = {8, 9};
    public static final int[] ELEVATOR_ULTRASONIC_PORTS = {7, 6};
    public static final int INTAKE_TALON_PORT = 6;

    public static final int ELEVATOR_TALON_PORT = 2;

    // Climber
    public static final int LIFTER_TALON_PORT = 0;
    //public static final int[] CLIMBER_SOLENOID_CHANNELS = {0,0};

    public static final int SHOOTER_PORT = 5;
    public static final int TURRET_PORT = 5;
    public static final int HOOD_PORT = 6;
    public static final int HOOD_SHOOTER_PORT = 7;

}
