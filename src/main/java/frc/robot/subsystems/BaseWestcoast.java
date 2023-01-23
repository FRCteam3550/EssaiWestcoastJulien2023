package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.westcoast.Odometrie;
import frc.robot.subsystems.westcoast.Simulation;
import frc.robot.util.Transmission;
import frc.robot.util.Unites;

public final class BaseWestcoast extends SubsystemBase {
    private static final Pose2d POSITION_DEPART =  new Pose2d(2, 2, Rotation2d.fromDegrees(0));

    private static final long COCHES_ENCODEUR_PAR_ROTATION = 2048;
    private static final double DIAMETRE_ROUE_METRES = 4 * Unites.POUCE;

    private static final int NO_CAN_MOTEUR_AVANT_GAUCHE = 4;
    private static final int NO_CAN_MOTEUR_AVANT_DROIT = 2;
    
    private static final double MODIFICATEUR_VITESSE_PRECIS = 0.2 ;
    private static final double MODIFICATEUR_VITESSE_RAPIDE = 1.0;

    public static final double K_S_VOLTS = 0.23112;
    public static final double K_V_VOLTS_SECONDES_PAR_METRES = 1.7074;
    public static final double K_A_VOLTS_SECONDES_CARRES_PAR_METRES = 0.36692;
    // Gain pour le contrôleur PID de WPILib (unités SI).
    public static final double K_P_VITESSE_VOLTS_SECONDES_PAR_METRES_WPI = 0.55913;
    // Gain pour les unités du Talon FX.
    public static final double K_P_VITESSE_VOLTS_100MS_PAR_COCHE_FALCON = 0.00076543;
    public static final double DIAMETRE_BASE_METRES = 0.77993 * Unites.METRE;
    public static final double VITESSE_MAX_METRES_PAR_SECONDES = 2;
    public static final double ACCELERATION_MAX_METRES_PAR_SECONDES_CARREES = 1;
    public static final double K_RAMSETE_B = 2;
    public static final double K_RAMSETE_ZETA = 0.7;

    private final WPI_TalonFX moteurGauche = new WPI_TalonFX(NO_CAN_MOTEUR_AVANT_GAUCHE);
    private final TalonFXSensorCollection capteursMoteurGauche = moteurGauche.getSensorCollection();
    private final WPI_TalonFX moteurDroit = new WPI_TalonFX(NO_CAN_MOTEUR_AVANT_DROIT);
    private final TalonFXSensorCollection capteursMoteurDroit = moteurDroit.getSensorCollection();
    private final DifferentialDrive chassis = new DifferentialDrive(moteurGauche, moteurDroit);
    private final Transmission transmissionRapide = new Transmission(COCHES_ENCODEUR_PAR_ROTATION, 1.0 / 5.13, DIAMETRE_ROUE_METRES);
    // private final Transmission transmissionPuissante = new Transmission(COCHES_ENCODEUR_PAR_ROTATION, 1.0 / 15.0, DIAMETRE_ROUE_METRES);
    private Transmission transmissionActive = transmissionRapide;
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveKinematics cinematiqueBase = new DifferentialDriveKinematics(DIAMETRE_BASE_METRES);
    private final Odometrie odometrie = new Odometrie(rotationRobot(), positionMoteurGaucheMetres(), positionMoteurDroitMetres(), POSITION_DEPART, cinematiqueBase);
    private final XboxController manette;

    private double modificateurVitesse = MODIFICATEUR_VITESSE_PRECIS;
    private boolean consigneQuadratique = false;

    private final Simulation sim = new Simulation(
        moteurGauche,
        moteurDroit,
        () -> transmissionActive,
        DIAMETRE_ROUE_METRES,
        DIAMETRE_BASE_METRES
    );

    public BaseWestcoast(XboxController manette) {
        this.manette = manette;

        moteurDroit.setInverted(true);
        moteurGauche.setNeutralMode(NeutralMode.Brake);
        moteurDroit.setNeutralMode(NeutralMode.Brake);
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        odometrie.update(
            rotationRobot(),
            positionMoteurGaucheMetres(), 
            -positionMoteurDroitMetres()
        );
    }

    @Override
    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }

    public Command piloter() {
        return
            new RunCommand(
                () -> chassis.arcadeDrive(-modificateurVitesse * manette.getRightY(), -modificateurVitesse * manette.getRightX(), consigneQuadratique),
                //() -> appliqueVoltage(-manette.getLeftY() * RobotController.getInputVoltage(), -manette.getRightY() * RobotController.getInputVoltage()),
                this
            )
            .andThen(chassis::stopMotor);
    }

    public Command trajectoireAuto() {
        return Commands.runOnce(() -> {
            var feedForward = new SimpleMotorFeedforward(
                K_S_VOLTS,
                K_V_VOLTS_SECONDES_PAR_METRES,
                K_A_VOLTS_SECONDES_CARRES_PAR_METRES
            );
    
            var contrainteVoltage =
                new DifferentialDriveVoltageConstraint(
                    feedForward,
                    cinematiqueBase,
                    10
                );
    
            var configurationTrajectoire =
                new TrajectoryConfig(
                    VITESSE_MAX_METRES_PAR_SECONDES,
                    ACCELERATION_MAX_METRES_PAR_SECONDES_CARREES
                )
                .setKinematics(cinematiqueBase)
                .addConstraint(contrainteVoltage);
    
            var trajectoireReferenceRobot = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                configurationTrajectoire
            );
    
            var trajectoireReferenceTerrain = trajectoireReferenceRobot
                .transformBy(odometrie.getPoseMeters().minus(trajectoireReferenceRobot.getInitialPose()));
    
            var ramseteCommand = new RamseteCommand(
                trajectoireReferenceTerrain,
                odometrie::getPoseMeters,
                new RamseteController(K_RAMSETE_B, K_RAMSETE_ZETA),
                feedForward,
                cinematiqueBase,
                this::vitesseRouesMetresParSeconde,
                new PIDController(K_P_VITESSE_VOLTS_SECONDES_PAR_METRES_WPI, 0, 0),
                new PIDController(K_P_VITESSE_VOLTS_SECONDES_PAR_METRES_WPI, 0, 0),
                this::appliqueVoltage,
                this
            );
    
            ramseteCommand.andThen(chassis::stopMotor).schedule();
        }, this);
    }

    public Command passeEnModePrecis() {
        return new InstantCommand(() -> {
            modificateurVitesse = MODIFICATEUR_VITESSE_PRECIS;
            consigneQuadratique = false;
        }, this);
    }

    public Command passeEnModeRapide() {
        return new InstantCommand(() -> {
            modificateurVitesse = MODIFICATEUR_VITESSE_RAPIDE;
            consigneQuadratique = true;
        }, this);
    }

    private Rotation2d rotationRobot() {
        if (Robot.isReal()) {
            return navx.getRotation2d();
        }
        else if (sim != null ) {
            return sim.rotationRobot();
        }
        return POSITION_DEPART.getRotation();
    }

    private double positionMoteurGaucheMetres() {
        if (Robot.isReal()) {
            return transmissionActive.distanceMetresPourCoches(capteursMoteurGauche.getIntegratedSensorPosition());
        }
        else if (sim != null) {
            return sim.positionMoteurGaucheMetres();
        }
        return 0;
    }

    private double positionMoteurDroitMetres() {
        if (Robot.isReal()) {
            return transmissionActive.distanceMetresPourCoches(capteursMoteurDroit.getIntegratedSensorPosition());
        }
        else if (sim != null) {
            return sim.positionMoteurDroitMetres();
        }
        return 0;
    }

    private DifferentialDriveWheelSpeeds vitesseRouesMetresParSeconde() {
        if (Robot.isReal()) {
            return new DifferentialDriveWheelSpeeds(
                transmissionActive.metresParSecondePourVitesse(capteursMoteurGauche.getIntegratedSensorVelocity()),
                transmissionActive.metresParSecondePourVitesse(capteursMoteurDroit.getIntegratedSensorVelocity())
            );
        }
        return new DifferentialDriveWheelSpeeds(
            sim.vitesseMoteurGaucheMetresParSeconde(),
            sim.vitesseMoteurDroitMetresParSeconde()
        );
    }

    /**
     * Voltage positif = vers l'avant.
     */
    private void appliqueVoltage(double moteurGaucheVolts, double moteurDroitVolts) {
        moteurGauche.setVoltage(moteurGaucheVolts);
        moteurDroit.setVoltage(moteurDroitVolts);
        chassis.feed();
    }

    /**
     * Les données du controlleur PID sont par défaut en nb de coches de l'encodeur, ce qui n'est pas très facile à visualiser.
     * On rajoute les données en mètres ici pour faciliter la calibration et le suivi.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("positionMoteurGaucheMetres", this::positionMoteurGaucheMetres, null);
        builder.addDoubleProperty("positionMoteurDroitMetres", this::positionMoteurDroitMetres, null);
        builder.addDoubleProperty("positionMoteurGaucheCoches", () -> capteursMoteurGauche.getIntegratedSensorPosition(), null);
        builder.addDoubleProperty("positionMoteurDroitCoches", () -> capteursMoteurDroit.getIntegratedSensorPosition(), null);
        builder.addDoubleProperty("angleGyro", () -> rotationRobot().getDegrees(), null);
        builder.addDoubleProperty("ajustementAngleGyro", navx::getAngleAdjustment, null);
    }
}
