package frc.robot.subsystems.westcoast;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathFollowing {
    private static final double K_P_VELOCITY_VSM = 0; // Mesurée: 0.68 ou 2.3

    private static final double MAX_VELOCITY_MS = 3;
    private static final double MAX_ACCELERATION_MS2 = 0.5;

    private static final double K_RAMSETE_B = 2; // Defaut: 2
    private static final double K_RAMSETE_ZETA = 0.7; // Defaut: 0.7

    private static final double MAX_VOLTS = 10;

    private static final Pose2d ZERO_POSE = new Pose2d();

    private final Field2d fieldTracker;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ChassisId.TRACK_WIDTH_M);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
        ChassisId.K_S_LIN_V,
        ChassisId.K_V_LIN_VSM,
        ChassisId.K_A_LIN_VS2M
    );
    private final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(feedForward, kinematics, MAX_VOLTS);
    private final TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig( MAX_VELOCITY_MS, MAX_ACCELERATION_MS2)
            .setKinematics(kinematics)
            .addConstraint(voltageConstraint);
    private final PIDController leftVelocityPid = new PIDController(K_P_VELOCITY_VSM, 0, 0);
    private final PIDController rightVelocityPid = new PIDController(K_P_VELOCITY_VSM, 0, 0);
    private final RamseteController ramsete = new RamseteController(K_RAMSETE_B, K_RAMSETE_ZETA);
    private final ControlerTelemetry telemetry = new ControlerTelemetry();
    private final Chassis chassis;

    public PathFollowing(Chassis chassis, Field2d fieldTracker) {
        this.chassis = chassis;
        this.fieldTracker = fieldTracker;
    }

    public DifferentialDriveKinematics cinematique() {
        return kinematics;
    }

    /**
     * Emmène le robot à la destination données, dans le référentiel du terrain.
     * @param destination une position du terrain, dans le référentiel du terrain.
     */
    public Command goTo(Pose2d destination) {
        var fieldRelativeTrajectory = TrajectoryGenerator.generateTrajectory(
            chassis.odometryEstimation(),  // On part de là où on est
            List.of(), // Aucun point intermédiaire: on va directement à la destination
            destination,
            trajectoryConfig
        );

        return follow(fieldRelativeTrajectory);
    }

    /**
     * Déplace le robot sur une trajectoire dans le référentiel du robot. Le robot part donc de là où il est.
     * @param points Points à suivre, dans le référentiel du robot.
     * @param finalPose Position d'arrivée, toujours dans le référentiel du robot.
     */
    public Command robotRelativeMove(List<Translation2d> points, Pose2d finalPose) {
        var robotRelativeTrajectory = TrajectoryGenerator.generateTrajectory(
            ZERO_POSE, // Dans le référentiel du robot, le point de départ, càd là où es le robot, est par définition à (0, 0)
            points,
            finalPose,
            trajectoryConfig
        );
        var robotToField = chassis.odometryEstimation().minus(ZERO_POSE);
        var fieldRelativeTrajectory = robotRelativeTrajectory.transformBy(robotToField);

        return follow(fieldRelativeTrajectory);
    }

    private Command follow(Trajectory trajectoire) {        
        return chassis.runOnce(() -> {
            fieldTracker.getObject("trajectoire").setTrajectory(trajectoire);
            ramsete.setEnabled(false);

            var ramseteCommand = new RamseteCommandWithTelemetry(
                trajectoire,
                chassis::odometryEstimation,
                ramsete,
                feedForward,
                kinematics,
                chassis::wheelSpeedsMS,
                leftVelocityPid,
                rightVelocityPid,
                chassis::applyVoltages,
                chassis
            );

            telemetry.observe(ramseteCommand);

            ramseteCommand.andThen(chassis::stopMotors).schedule();
        }).withName("trajectoireAuto");
    }        
}
