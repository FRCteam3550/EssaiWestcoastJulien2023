package frc.robot.util;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class DashboardOdometrie implements Sendable {
    private final DifferentialDriveOdometry odometrie;

    public DashboardOdometrie(DifferentialDriveOdometry odometrie) {
        this.odometrie = odometrie;
        SendableRegistry.add(this, "Odometrie");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(odometrie.getClass().getSimpleName());
        builder.addDoubleProperty("xMetres", () -> odometrie.getPoseMeters().getX(), null);
        builder.addDoubleProperty("yMetres", () -> odometrie.getPoseMeters().getY(), null);
        builder.addDoubleProperty("angleDegres", () -> odometrie.getPoseMeters().getRotation().getDegrees(), null);

    }
}
