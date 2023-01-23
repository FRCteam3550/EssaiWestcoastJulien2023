package frc.robot.subsystems;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

public class BaseWestcoastTest {
    private static final TrajectoryConfig configTrajectoire = new TrajectoryConfig(
        2,
        1
    );

    @Test
    void trajectoireDansReferenceRobot() {
        var robotPose = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
        var trajectoireReferenceRobot = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            // Pass config
            configTrajectoire
        );
        var trajectoireReferenceTerrain = trajectoireReferenceRobot.transformBy(robotPose.minus(trajectoireReferenceRobot.getInitialPose()));
        
        assertEquals(robotPose.getTranslation(), trajectoireReferenceTerrain.getInitialPose().getTranslation());
        assertEquals(robotPose.getRotation(), trajectoireReferenceTerrain.getInitialPose().getRotation());
    }    
}
