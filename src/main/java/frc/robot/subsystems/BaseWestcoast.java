package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.westcoast.Chassis;
import frc.robot.subsystems.westcoast.WestcoastOdometry;
import frc.robot.subsystems.westcoast.Simulation;
import frc.robot.subsystems.westcoast.PathFollowing;
import frc.robot.subsystems.westcoast.ChassisId;
import frc.robot.util.Navx;
import frc.robot.util.Gearing;

public final class BaseWestcoast extends SubsystemBase implements Chassis {
    private static final Pose2d DEVANT_TAG_0 = new Pose2d(1.8, 3.5, Rotation2d.fromDegrees(90));

    private static final long COCHES_ENCODEUR_PAR_ROTATION = 2048;
    private static final int NO_CAN_MOTEUR_AVANT_GAUCHE = 4;
    private static final int NO_CAN_MOTEUR_AVANT_DROIT = 2;
    
    private static final double PRECISION_DRIVE_LIMITATION = 0.3;
    private static final double SPEEDY_DRIVE_LIMITATION = 1.0;

    private final Field2d terrain = new Field2d();
    private final AHRS navx = Navx.newReadyNavx();
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(NO_CAN_MOTEUR_AVANT_GAUCHE);
    private final TalonFXSensorCollection leftMotorSensors = leftMotor.getSensorCollection();
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(NO_CAN_MOTEUR_AVANT_DROIT);
    private final TalonFXSensorCollection rightMotorSensors = rightMotor.getSensorCollection();
    private final DifferentialDrive chassis = new DifferentialDrive(leftMotor, rightMotor);
    private final Gearing quickGearing = new Gearing(COCHES_ENCODEUR_PAR_ROTATION, 5.13 / 1.0, ChassisId.WHEEL_DIAMETER_M);
    // private final Gearing powerfulGearing = new Gearing(COCHES_ENCODEUR_PAR_ROTATION, 15.0 / 1.0, ChassisId.WHEEL_DIAMETER_M);
    private Gearing activeGearing = quickGearing;
    private final PathFollowing pathFollowing = new PathFollowing(this, terrain);
    private final XboxController gamepad;

    private double speedInputLimitation = PRECISION_DRIVE_LIMITATION;
    private boolean useQuadraticInputs = false;
    private DifferentialDriveWheelSpeeds lastSpeeds = new DifferentialDriveWheelSpeeds();

    private final Simulation sim = new Simulation(leftMotor, rightMotor, () -> activeGearing);
    private final WestcoastOdometry odometry = new WestcoastOdometry(rotationRobot(), positionMoteurGaucheMetres(), positionMoteurDroitMetres(), pathFollowing.cinematique(), terrain);

    public BaseWestcoast(XboxController manette) {
        this.gamepad = manette;

        rightMotor.setInverted(true);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        rightMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        leftMotor.configVelocityMeasurementWindow(1);
        rightMotor.configVelocityMeasurementWindow(1);

        SmartDashboard.putData(this);
        SmartDashboard.putData(terrain);

        setDefaultCommand(drive());
    }
    
    @Override
    public void periodic() {
        odometry.update(
            rotationRobot(),
            positionMoteurGaucheMetres(), 
            positionMoteurDroitMetres()
        );
    }

    @Override
    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }

    public Command drive() {
        return 
            run(() -> 
                chassis.arcadeDrive(
                    -speedInputLimitation * gamepad.getRightY(),
                    -speedInputLimitation * gamepad.getRightX(),
                    useQuadraticInputs
                )
            )
            .andThen(chassis::stopMotor)
            .withName("piloter");
    }

    public Command goToFrontOfTag0() {
        return pathFollowing.goTo(DEVANT_TAG_0);
    }

    public Command zigzag() {
        return pathFollowing.robotRelativeMove(
            List.of(
                new Translation2d(0.5, 0.5),
                new Translation2d(1, -0.5)
            ),
            new Pose2d(1.5, 0, Rotation2d.fromDegrees(0))
        );
    }

    public Command passeEnModePrecis() {
        return runOnce(() -> {
            speedInputLimitation = PRECISION_DRIVE_LIMITATION;
            useQuadraticInputs = false;
        });
    }

    public Command passeEnModeRapide() {
        return runOnce(() -> {
            speedInputLimitation = SPEEDY_DRIVE_LIMITATION;
            useQuadraticInputs = true;
        });
    }

    public Command enableCameraEstimate() {
        return runOnce(odometry::activateCameraEstimations);
    }
    
    public Command disableCameraEstimate() {
        return runOnce(odometry::deactivateCameraEstimations);
    }

    public Pose2d odometryEstimation() {
        return odometry.getPoseM();
    }

    private Rotation2d rotationRobot() {
        if (Robot.isReal()) {
            // navx.getRotation2d() is continuous and not in the [-180, +180] range
            return Rotation2d.fromDegrees(-navx.getYaw());
        }
        return sim.robotRotation();
    }

    // Avancer = valeurs positives
    private double positionMoteurGaucheMetres() {
        if (Robot.isReal()) {
            return activeGearing.distanceMForTicks(leftMotorSensors.getIntegratedSensorPosition());
        }
        return sim.leftMotorPositionM();
    }

    // Avancer = valeurs positives
    private double positionMoteurDroitMetres() {
        if (Robot.isReal()) {
            return activeGearing.distanceMForTicks(-rightMotorSensors.getIntegratedSensorPosition());
        }
        return sim.rightMotorPositionM();
    }

    public DifferentialDriveWheelSpeeds wheelSpeedsMS() {
        DifferentialDriveWheelSpeeds vitesses;
        if (Robot.isReal()) {
            // Avancer = vitesses positives, donc il faut inverser la valeur de l'encodeur droit
            vitesses = new DifferentialDriveWheelSpeeds(
                activeGearing.speedMSForSpeedT100MS(leftMotorSensors.getIntegratedSensorVelocity()),
                activeGearing.speedMSForSpeedT100MS(-rightMotorSensors.getIntegratedSensorVelocity())
            );
        } else {
            vitesses = sim.wheelSpeedsMS();
        }
        lastSpeeds = vitesses;
        return vitesses;
}

    /**
     * Voltage positif = vers l'avant.
     */
    public void applyVoltages(double moteurGaucheVolts, double moteurDroitVolts) {
        leftMotor.setVoltage(moteurGaucheVolts);
        rightMotor.setVoltage(moteurDroitVolts); // Signe fonctionne, car nous avons inversé ce moteur dans le constructeur.
        chassis.feed();
    }

    public void stopMotors() {
        chassis.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("positionMoteurGaucheMetres", this::positionMoteurGaucheMetres, null);
        builder.addDoubleProperty("positionMoteurDroitMetres", this::positionMoteurDroitMetres, null);
        builder.addDoubleProperty("angleGyro", () -> rotationRobot().getDegrees(), null);
        builder.addDoubleProperty("ajustementAngleGyro", navx::getAngleAdjustment, null);
        builder.addDoubleProperty("vitMotGaucheMS", () -> lastSpeeds.leftMetersPerSecond, null);
        builder.addDoubleProperty("vitMotDroitMS", () -> lastSpeeds.rightMetersPerSecond, null);
    }  
}
