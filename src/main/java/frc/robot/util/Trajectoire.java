package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class Trajectoire {
    private final Deque<Pose2d> positions = new ArrayDeque<>();

    public Trajectoire(Pose2d positionInitiale) {
        positions.add(positionInitiale);
    }

    public Trajectoire avance(double distanceMetres) {
        var depart = positions.getLast();
        return ajouteTransformation(
            new Translation2d(1, depart.getRotation()),
            new Rotation2d()
        );
    }

    public Trajectoire tourne(double angleDegres) {
        return ajouteTransformation(
            new Translation2d(),
            Rotation2d.fromDegrees(angleDegres)
        );
    }

    private Trajectoire ajouteTransformation(Translation2d translation, Rotation2d rotation) {
        var depart = positions.getLast();
        var transformation = new Transform2d(translation, rotation);
        positions.add(depart.plus(transformation));
        return this;
    }

    public Trajectory finalise(TrajectoryConfig configurationTrajectoire) {
        return TrajectoryGenerator.generateTrajectory(
            // Start
            positions.getFirst(),
            // Pass through these interior waypoints
            positions
                .stream()
                .skip(1)
                .limit(positions.size() - 2)
                .map(t -> t.getTranslation()).toList(),
            // End
            positions.getLast(),
            // Pass config
            configurationTrajectoire
        );
    }
}
