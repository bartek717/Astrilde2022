package frc.robot;

public class BangBangController extends edu.wpi.first.math.controller.BangBangController {
    
    private final double power;

    public BangBangController(double power, double tolerance) {
        super(tolerance);
        this.power = power;
    }

    public double calculate(double measurement, double setpoint) {
      double original = super.calculate(measurement, setpoint);
      return original == 1 ? power : 0;
    }
}
