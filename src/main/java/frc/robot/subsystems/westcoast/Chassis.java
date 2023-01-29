package frc.robot.subsystems.westcoast;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Chassis extends Subsystem {
  Pose2d odometryEstimation();
  DifferentialDriveWheelSpeeds wheelSpeedsMS();
  void applyVoltages(double leftMotorV, double rightMotorV);
  void stopMotors();
}
