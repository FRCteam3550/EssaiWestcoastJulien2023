package frc.robot.subsystems.westcoast;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.util.Transmission;

/**
 * Regroupe tout le code pour la simulation de la base Westcoast.
 */
public class Simulation {
    private final WPI_TalonFX moteurGauche;
    private final WPI_TalonFX moteurDroit;
    private final TalonFXSimCollection simMoteurGauche;
    private final TalonFXSimCollection simMoteurDroit;
    private final DifferentialDrivetrainSim sim;
    private final Supplier<Transmission> transmission;
    private final Timer timer = new Timer();

    public Simulation(
        WPI_TalonFX moteurGauche,
        WPI_TalonFX moteurDroit,
        Supplier<Transmission> transmission,
        double diametreRoueMetres,
        double diametreBaseMetres) {
        this.moteurGauche = moteurGauche;
        this.moteurDroit = moteurDroit;
        this.transmission = transmission;
        this.simMoteurGauche = moteurGauche.getSimCollection();
        this.simMoteurDroit = moteurDroit.getSimCollection();
        this.sim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(1), // Un moteur par coté
            1 / transmission.get().getReduction(),
            3, // MOI de 3 Kg m^2, puisque le chassis est vide
            20, // TODO: Masse du robot: 20 Kg
            diametreRoueMetres / 2, // Rayon roues
            diametreBaseMetres,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) // Deviations standards de mesure
        );
        this.timer.start();
    }

    public Rotation2d rotationRobot() {
        return sim.getHeading();
    }

    public double positionMoteurGaucheMetres() {
        return sim.getLeftPositionMeters();
    }

    public double positionMoteurDroitMetres() {
        return sim.getRightPositionMeters();
    }

    public double vitesseMoteurGaucheMetresParSeconde() {
        return sim.getLeftVelocityMetersPerSecond();
    }

    public double vitesseMoteurDroitMetresParSeconde() {
        return sim.getRightVelocityMetersPerSecond();
    }

    public void simulationPeriodic() {
        // La simulation suppose qu'un voltage positif est pour avancer, donc il faut inverser pour le moteur droit. Mais comme on fait l'inversion au niveau matériel, les voltages retournés par les encodeurs ne sont pas inversés.

        // On "sauve" la transmission courante pour utiliser la même durant toute la durée de la méthode, au cas où elle
        // change pendant.
        var transmissionActive = transmission.get();
        sim.setCurrentGearing(1 / transmissionActive.getReduction());

        // On dit à la simulation quels sont les voltages que l'on est en train d'appliquer aux moteurs.
        sim.setInputs(
            moteurGauche.get() * RobotController.getInputVoltage(),
            moteurDroit.get() * RobotController.getInputVoltage()
        );

        // Combien de temps s'est écoulé depuis la dernière fois
        sim.update(timer.get());
        timer.reset();

        // On écoute la simulation pour savoir quels devraient être les valeurs des capteurs.
        simMoteurGauche.setBusVoltage(RobotController.getBatteryVoltage());
        simMoteurGauche.setIntegratedSensorRawPosition((int)transmissionActive.cochesPourDistance(sim.getLeftPositionMeters()));
        simMoteurGauche.setIntegratedSensorVelocity((int)transmissionActive.cochesPar100msPourVitesse(sim.getLeftVelocityMetersPerSecond()));

        simMoteurDroit.setBusVoltage(RobotController.getBatteryVoltage());
        simMoteurDroit.setIntegratedSensorRawPosition((int)transmissionActive.cochesPourDistance(sim.getRightPositionMeters()));
        simMoteurDroit.setIntegratedSensorVelocity((int)transmissionActive.cochesPar100msPourVitesse(sim.getRightVelocityMetersPerSecond()));
    }    
}
