package frc.robot.subsystems.westcoast;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometrie implements Sendable {
    private final Field2d terrain = new Field2d();
    private final AprilTagFieldLayout dispositionTerrain = new AprilTagFieldLayout(List.of(
        new AprilTag(0, new Pose3d(2, 4.4, 0.8, new Rotation3d(0, 0, Math.toRadians(-90))))),
        4.5,
        4.4
    );
    //private final AprilTagFieldLayout dispositionTerrain = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    private final PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    private final Transform3d robotACamera = new Transform3d(
        new Translation3d(0.5, 0.0, 0.2),
        new Rotation3d(0,0,0)
    ); // TODO: à mesurer
    private final RobotPoseEstimator estimateurPositionCamera = new RobotPoseEstimator(
        dispositionTerrain,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        List.of(Pair.of(camera, robotACamera))
    );
    private final DifferentialDrivePoseEstimator estimateurPositionEtat;
    private Pose2d dernierEstime = new Pose2d();
    private double tempsDernierEstimeCamera = 0;

    public Odometrie(
        Rotation2d rotationRobot,
        double positionMoteurGaucheMetres,
        double positionMoteurDroitMetres,
        Pose2d positionDepart,
        DifferentialDriveKinematics cinematiqueBase) {

        estimateurPositionEtat = new DifferentialDrivePoseEstimator(cinematiqueBase, rotationRobot, positionMoteurGaucheMetres, positionMoteurDroitMetres, positionDepart);
        SendableRegistry.add(this, "Odometrie");
        SmartDashboard.putData(this);
        SmartDashboard.putData(terrain);
    }

    public void update(Rotation2d rotationRobot, double positionMoteurGaucheMetres, double positionMoteurDroitMetres) {
        estimateurPositionEtat.update(
            rotationRobot,
            positionMoteurGaucheMetres, 
            -positionMoteurDroitMetres
        );
        estimateurPositionCamera.setReferencePose(estimateurPositionEtat.getEstimatedPosition());
        var optionEstimation = estimateurPositionCamera.update();
        if (optionEstimation.isPresent() && optionEstimation.get().getFirst() != null) {
            var estimation = optionEstimation.get();
            estimateurPositionEtat.addVisionMeasurement(estimation.getFirst().toPose2d(), estimation.getSecond());
            tempsDernierEstimeCamera = estimation.getSecond();
        }
        dernierEstime = estimateurPositionEtat.getEstimatedPosition();
        terrain.setRobotPose(dernierEstime);
    }

    public Pose2d getPoseMeters() {
        dernierEstime = estimateurPositionEtat.getEstimatedPosition();
        return dernierEstime;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getClass().getSimpleName());
        builder.addDoubleProperty("xMetres", () -> this.dernierEstime.getX(), null);
        builder.addDoubleProperty("yMetres", () -> this.dernierEstime.getY(), null);
        builder.addDoubleProperty("angleDegres", () -> dernierEstime.getRotation().getDegrees(), null);
        builder.addDoubleProperty("tempsDernierEstimeCamera", () -> this.tempsDernierEstimeCamera, null);
    }
}
 