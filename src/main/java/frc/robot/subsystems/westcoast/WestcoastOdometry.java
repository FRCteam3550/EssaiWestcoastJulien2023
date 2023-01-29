package frc.robot.subsystems.westcoast;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WestcoastOdometry implements Sendable {
    private static final Pose2d POSITION_DEPART =  new Pose2d(2, 2, Rotation2d.fromDegrees(0));

    private final Field2d fieldTracker;
    //private final AprilTagFieldLayout dispositionTerrain = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    private final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(
        List.of(
            new AprilTag(0, new Pose3d(1.73, 4.4, 0.8, new Rotation3d(0, 0, Math.toRadians(-90))))
        ),
        4.5,
        4.4
    );
    private final Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.523, 0.258 , 0.401),
        new Rotation3d(0,0,0)
    );
    private final PhotonPoseEstimator cameraPoseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        new PhotonCamera("Microsoft_LifeCam_HD-3000"),
        robotToCamera
    );
    private final DifferentialDrivePoseEstimator poseEstimator;

    private Pose2d lastPoseEstimation = new Pose2d();
    private double distanceDifference = 0;
    private Timer lastCameraEstimateTime = new Timer();
    private boolean areCameraEstimationsActive = true;

    public WestcoastOdometry(
        Rotation2d robotRotation,
        double leftMotorPositionM,
        double rightMotorPositionM,
        DifferentialDriveKinematics kinematics,
        Field2d fieldTracker) {

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, robotRotation, leftMotorPositionM, rightMotorPositionM, POSITION_DEPART);
        SendableRegistry.add(this, "Odometrie");
        SmartDashboard.putData(this);

        lastCameraEstimateTime.reset();
        lastCameraEstimateTime.start();

        this.fieldTracker = fieldTracker;
        fieldTracker.getObject("tag").setPose(fieldLayout.getTagPose(0).get().toPose2d());
    }

    // Positive values are going forward 
    public void update(Rotation2d robotRotation, double leftMotorPositionM, double rightMotorPositionM) {
        poseEstimator.update(
            robotRotation,
            leftMotorPositionM, 
            rightMotorPositionM
        );

        if (areCameraEstimationsActive) {
            var estimationWithoutCamera = poseEstimator.getEstimatedPosition();
            cameraPoseEstimator.setReferencePose(estimationWithoutCamera);

            var maybeEstimation = cameraPoseEstimator.update();
            if (maybeEstimation.isPresent() && maybeEstimation.get().estimatedPose != null) {
                var estimation = maybeEstimation.get();
                poseEstimator.addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds);
                
                lastCameraEstimateTime.reset();
                lastCameraEstimateTime.start();
                var estimationWithCamera = poseEstimator.getEstimatedPosition();
                var diff = estimationWithCamera.minus(estimationWithoutCamera);
                distanceDifference = Math.sqrt(diff.getX()*diff.getX() + diff.getY()*diff.getY());
            }
        }

        lastPoseEstimation = poseEstimator.getEstimatedPosition();
        fieldTracker.setRobotPose(lastPoseEstimation);
    }   

    public Pose2d getPoseM() {
        lastPoseEstimation = poseEstimator.getEstimatedPosition();
        return lastPoseEstimation;
    }

    public void activateCameraEstimations() {
        areCameraEstimationsActive = true;
    }
    
    public void deactivateCameraEstimations() {
        areCameraEstimationsActive = false;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getClass().getSimpleName());
        builder.addDoubleProperty("xMetres", () -> this.lastPoseEstimation.getX(), null);
        builder.addDoubleProperty("yMetres", () -> this.lastPoseEstimation.getY(), null);
        builder.addDoubleProperty("angleDegres", () -> lastPoseEstimation.getRotation().getDegrees(), null);
        builder.addDoubleProperty("diffDistance", () -> this.distanceDifference, null);
        builder.addBooleanProperty("positionSecuritaire", () -> this.lastCameraEstimateTime.get() < 10, null);
        builder.addBooleanProperty("cameraActivee", () -> this.areCameraEstimationsActive, null);
    }
}
 