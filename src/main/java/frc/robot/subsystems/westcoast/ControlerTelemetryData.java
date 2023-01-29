package frc.robot.subsystems.westcoast;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public interface ControlerTelemetryData {
  DifferentialDriveWheelSpeeds reference();
  DifferentialDriveWheelSpeeds target();
  DifferentialDriveWheelSpeeds actual();
  Pair<Double, Double> output();

  static final Pair<Double, Double> ZERO_OUTPUT = new Pair<>(0.0d, 0.0d);
  static final DifferentialDriveWheelSpeeds ZERO_SPEEDS = new DifferentialDriveWheelSpeeds(0.0, 0.0);
  static final ControlerTelemetryData ZERO = new ControlerTelemetryData() {
    
      @Override
    public DifferentialDriveWheelSpeeds reference() {
      return ZERO_SPEEDS;
    }

    @Override
    public DifferentialDriveWheelSpeeds target() {
      return ZERO_SPEEDS;
    }

    @Override
    public DifferentialDriveWheelSpeeds actual() {
      return ZERO_SPEEDS;
    }
    
    @Override
    public Pair<Double, Double> output() {
      return ZERO_OUTPUT;
    }
  };
}
