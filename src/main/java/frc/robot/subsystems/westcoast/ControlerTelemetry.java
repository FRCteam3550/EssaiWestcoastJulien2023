package frc.robot.subsystems.westcoast;

import java.util.Objects;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlerTelemetry implements Sendable {
  private ControlerTelemetryData observedControler = ControlerTelemetryData.ZERO;

  public ControlerTelemetry() {
    SendableRegistry.add(this, "ControlerTelemetry");
    SmartDashboard.putData(this);
  }

  public void observe(ControlerTelemetryData controler) {
    observedControler = Objects.requireNonNull(controler);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(getClass().getSimpleName());
    
    builder.addDoubleProperty("leftRefVelocity", () -> observedControler.reference().leftMetersPerSecond, null);
    builder.addDoubleProperty("rightRefVelocity", () -> observedControler.reference().rightMetersPerSecond, null);
    builder.addDoubleProperty("leftTargetVelocity", () -> observedControler.target().leftMetersPerSecond, null);
    builder.addDoubleProperty("rightTargetVelocity", () -> observedControler.target().rightMetersPerSecond, null);
    builder.addDoubleProperty("leftActualVelocity", () -> observedControler.actual().leftMetersPerSecond, null);
    builder.addDoubleProperty("rightActualVelocity", () -> observedControler.actual().rightMetersPerSecond, null);
    builder.addDoubleProperty("leftOutput", () -> observedControler.output().getFirst(), null);
    builder.addDoubleProperty("rightOutput", () -> observedControler.output().getSecond(), null);
  }
}
