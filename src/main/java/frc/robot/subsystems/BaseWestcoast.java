package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.westcoast.Odometrie;
import frc.robot.subsystems.westcoast.Simulation;
import frc.robot.util.Transmission;
import frc.robot.util.Unites;

public final class BaseWestcoast extends SubsystemBase {
    private static final Pose2d POSITION_DEPART =  new Pose2d(2, 2, Rotation2d.fromDegrees(0));

    private static final long COCHES_ENCODEUR_PAR_ROTATION = 2048;
    private static final double DIAMETRE_ROUE_METRES = 0.1017 * Unites.METRE; // Circonférence: 0.3195

    private static final int NO_CAN_MOTEUR_AVANT_GAUCHE = 4;
    private static final int NO_CAN_MOTEUR_AVANT_DROIT = 2;
    
    private static final double MODIFICATEUR_VITESSE_PRECIS = 0.2 ;
    private static final double MODIFICATEUR_VITESSE_RAPIDE = 1.0;

    // Voir résultats caractérisation: https://docs.google.com/spreadsheets/d/1WEG3WhIzJu3VFnJeH4EQ7ALOVoR8XWWcQZ5B1EWVP3w/edit?usp=sharing
    public static final double K_S_VOLTS = 0.24; // Mesurée: 0.24
    public static final double K_V_VOLTS_SECONDES_PAR_METRES = 1.65;
    public static final double K_A_VOLTS_SECONDES_CARRES_PAR_METRES = 0.45;
    // Gain pour le contrôleur PID de WPILib (unités SI).
    public static final double K_P_VITESSE_VOLTS_SECONDES_PAR_METRES_WPI = 2.3; // Mesurée: 0.68 ou 2.3
    public static final double DIAMETRE_BASE_METRES = 0.77993 * Unites.METRE;
    public static final double VITESSE_MAX_METRES_PAR_SECONDES = 3;
    public static final double ACCELERATION_MAX_METRES_PAR_SECONDES_CARREES = 1;
    public static final double K_RAMSETE_B = 2; // Defaut: 2
    public static final double K_RAMSETE_ZETA = 0.7; // Defaut: 0.7

    private final AHRS navx = newNavx();
    private final WPI_TalonFX moteurGauche = new WPI_TalonFX(NO_CAN_MOTEUR_AVANT_GAUCHE);
    private final TalonFXSensorCollection capteursMoteurGauche = moteurGauche.getSensorCollection();
    private final WPI_TalonFX moteurDroit = new WPI_TalonFX(NO_CAN_MOTEUR_AVANT_DROIT);
    private final TalonFXSensorCollection capteursMoteurDroit = moteurDroit.getSensorCollection();
    private final DifferentialDrive chassis = new DifferentialDrive(moteurGauche, moteurDroit);
    private final Transmission transmissionRapide = new Transmission(COCHES_ENCODEUR_PAR_ROTATION, 1.0 / 5.13, DIAMETRE_ROUE_METRES);
    // private final Transmission transmissionPuissante = new Transmission(COCHES_ENCODEUR_PAR_ROTATION, 1.0 / 15.0, DIAMETRE_ROUE_METRES);
    private Transmission transmissionActive = transmissionRapide;
    private final DifferentialDriveKinematics cinematiqueBase = new DifferentialDriveKinematics(DIAMETRE_BASE_METRES);
    private final Odometrie odometrie = new Odometrie(rotationRobot(), positionMoteurGaucheMetres(), positionMoteurDroitMetres(), POSITION_DEPART, cinematiqueBase);
    private final XboxController manette;

    private double modificateurVitesse = MODIFICATEUR_VITESSE_PRECIS;
    private boolean consigneQuadratique = false;
    private double derniereVitesseGaucheMetresSeconde = 0;
    private double consigneVitesseGaucheMetresSeconde = 0;
    private double derniereVitesseDroiteMetresSeconde = 0;
    private double consigneVitesseDroiteMetresSeconde = 0;

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
        moteurGauche.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        moteurDroit.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        moteurGauche.configVelocityMeasurementWindow(1);
        moteurDroit.configVelocityMeasurementWindow(1);

        SmartDashboard.putData(this);

        setDefaultCommand(piloter());
    }

    /*
     * Lorsque nouvellement initialisé, le navx donne un angle de 0 pendant plusieurs cycles de mesures au lieu de la vraie valeur.
     * Or, il faut la vraie valeur pour initialiser l'odométrie. Je n'ai pas trouvé de méthode m'indiquant que l'initialisation est
     * terminée, donc j'attends jusqu'à ce qu'une valeur différente de 0 apparaisse.
     */
    private static AHRS newNavx() {
        var navx = new AHRS(SPI.Port.kMXP);

        if (Robot.isReal()) {
            while (Math.abs(navx.getYaw()) < 0.0001) {
                try {
                    Thread.sleep(20);
                }
                catch(InterruptedException ie) {
                    // Do nothing.
                }
            }
        }
        return navx;
    }
    
    @Override
    public void periodic() {
        odometrie.update(
            rotationRobot(),
            positionMoteurGaucheMetres(), 
            positionMoteurDroitMetres()
        );
    }

    @Override
    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }

    public Command piloter() {
        return 
            run(() -> 
                chassis.arcadeDrive(
                    -modificateurVitesse * manette.getRightY(),
                    -modificateurVitesse * manette.getRightX(),
                    consigneQuadratique
                )
            )
            .andThen(chassis::stopMotor)
            .withName("piloter");
    }

    public Command trajectoireAuto() {
        return runOnce(() -> {
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
    
            // var cibleRobot = new Pose2d(1.5, 0, Rotation2d.fromDegrees(0));
            // var trajectoireReferenceRobot = TrajectoryGenerator.generateTrajectory(
            //     new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            //     List.of(
            //         // new Translation2d(1, 0.2),
            //         // new Translation2d(1.5, 0)
            //     ),
            //     cibleRobot,
            //     configurationTrajectoire
            // );
    
            // var cibleRobot = new Pose2d(1.2, 0, Rotation2d.fromDegrees(0));
            var trajectoireReferenceTerrain = TrajectoryGenerator.generateTrajectory(
                odometrie.getPoseMeters(),
                List.of(
                ),
                new Pose2d(1.8, 3.5, Rotation2d.fromDegrees(90)),
                configurationTrajectoire
            );

            // var robotATerrain = odometrie.getPoseMeters().minus(trajectoireReferenceRobot.getInitialPose());
            // var trajectoireReferenceTerrain = trajectoireReferenceRobot
            //     .transformBy(robotATerrain);
            // cible = cibleRobot.transformBy(robotATerrain);
            odometrie.setTrajectoire(trajectoireReferenceTerrain);

            var pidGauche = new PIDController(K_P_VITESSE_VOLTS_SECONDES_PAR_METRES_WPI, 0, 0);
            var pidDroit = new PIDController(K_P_VITESSE_VOLTS_SECONDES_PAR_METRES_WPI, 0, 0);
            var ramsete = new RamseteController(K_RAMSETE_B, K_RAMSETE_ZETA);
            ramsete.setEnabled(true);
    
            var ramseteCommand = new RamseteCommand(
                trajectoireReferenceTerrain,
                odometrie::getPoseMeters,
                ramsete,
                feedForward,
                cinematiqueBase,
                this::vitesseRouesMetresParSeconde,
                pidGauche,
                pidDroit,
                (voltGauche, voltDroit) -> {
                    consigneVitesseGaucheMetresSeconde = pidGauche.getSetpoint();
                    consigneVitesseDroiteMetresSeconde = pidDroit.getSetpoint();
                    appliqueVoltage(voltGauche, voltDroit);
                },
                this
            );
    
            ramseteCommand.andThen(chassis::stopMotor).schedule();
        }).withName("trajectoireAuto");
    }

    public Command passeEnModePrecis() {
        return runOnce(() -> {
            modificateurVitesse = MODIFICATEUR_VITESSE_PRECIS;
            consigneQuadratique = false;
        });
    }

    public Command passeEnModeRapide() {
        return runOnce(() -> {
            modificateurVitesse = MODIFICATEUR_VITESSE_RAPIDE;
            consigneQuadratique = true;
        });
    }

    public Command enableCameraEstimate() {
        return runOnce(odometrie::activeEstimeParCamera);
    }
    
    public Command disableCameraEstimate() {
        return runOnce(odometrie::desactiveEstimeParCamera);
    }

    private Rotation2d rotationRobot() {
        if (Robot.isReal()) {
            // navx.getRotation2d() is continuous and not in the [-180, +180] range
            return Rotation2d.fromDegrees(-navx.getYaw());
        }
        else if (sim != null ) {
            return sim.rotationRobot();
        }
        return POSITION_DEPART.getRotation();
    }

    // Avancer = valeurs positives
    private double positionMoteurGaucheMetres() {
        if (Robot.isReal()) {
            return transmissionActive.distanceMetresPourCoches(capteursMoteurGauche.getIntegratedSensorPosition());
        }
        else if (sim != null) {
            return sim.positionMoteurGaucheMetres();
        }
        return 0;
    }

    // Avancer = valeurs positives
    private double positionMoteurDroitMetres() {
        if (Robot.isReal()) {
            return transmissionActive.distanceMetresPourCoches(-capteursMoteurDroit.getIntegratedSensorPosition());
        }
        else if (sim != null) {
            return sim.positionMoteurDroitMetres();
        }
        return 0;
    }

    private DifferentialDriveWheelSpeeds vitesseRouesMetresParSeconde() {
        if (Robot.isReal()) {
            var result = new DifferentialDriveWheelSpeeds(
                transmissionActive.metresParSecondePourVitesse(capteursMoteurGauche.getIntegratedSensorVelocity()),
                transmissionActive.metresParSecondePourVitesse(-capteursMoteurDroit.getIntegratedSensorVelocity())
            );
            derniereVitesseGaucheMetresSeconde = result.leftMetersPerSecond;
            derniereVitesseDroiteMetresSeconde = result.rightMetersPerSecond;
            return result;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        //super.initSendable(builder);

        builder.addDoubleProperty("positionMoteurGaucheMetres", this::positionMoteurGaucheMetres, null);
        builder.addDoubleProperty("positionMoteurDroitMetres", this::positionMoteurDroitMetres, null);
        builder.addDoubleProperty("angleGyro", () -> rotationRobot().getDegrees(), null);
        builder.addDoubleProperty("ajustementAngleGyro", navx::getAngleAdjustment, null);
        builder.addDoubleProperty("vitMotGaucheMS", () -> derniereVitesseGaucheMetresSeconde, null);
        builder.addDoubleProperty("vitMotDroitMS", () -> derniereVitesseDroiteMetresSeconde, null);
        builder.addDoubleProperty("consigneVitGaucheMS", () -> consigneVitesseGaucheMetresSeconde, null);
        builder.addDoubleProperty("consigneVitGDroitMS", () -> consigneVitesseDroiteMetresSeconde, null);
    }
}
