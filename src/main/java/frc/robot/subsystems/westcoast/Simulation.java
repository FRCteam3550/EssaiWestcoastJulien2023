package frc.robot.subsystems.westcoast;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.util.Gearing;

/**
 * Regroupe tout le code pour la simulation de la base Westcoast.
 */
public class Simulation {
    private final WPI_TalonFX leftMotor;
    private final WPI_TalonFX rightMotor;
    private final TalonFXSimCollection leftMotorSim;
    private final TalonFXSimCollection rightMotorSim;
    private final DifferentialDrivetrainSim sim;
    private final Supplier<Gearing> transmission;
    private final Timer timer = new Timer();

    public Simulation(
        WPI_TalonFX leftMotor,
        WPI_TalonFX rightMotor,
        Supplier<Gearing> gearing) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.transmission = gearing;
        this.leftMotorSim = leftMotor.getSimCollection();
        this.rightMotorSim = rightMotor.getSimCollection();
        // this.sim = new DifferentialDrivetrainSim(
        //     DCMotor.getFalcon500(1), // Un moteur par coté
        //     1 / transmission.get().getReduction(),
        //     3, // MOI de 3 Kg m^2, puisque le chassis est vide
        //     20, // TODO: Masse du robot: 20 Kg
        //     diametreRoueMetres / 2, // Rayon roues
        //     diametreBaseMetres,
        //     VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) // Deviations standards de mesure
        // );
        this.sim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                ChassisId.K_V_LIN_VSM,
                ChassisId.K_A_LIN_VS2M,
                ChassisId.K_V_ROT_VSM,
                ChassisId.K_A_ROT_VS2M,
                ChassisId.TRACK_WIDTH_M
            ),
            DCMotor.getFalcon500(1),
            gearing.get().getGearingRatioEncoderShaftTurnsOverWheelShaftTurns(),
            ChassisId.TRACK_WIDTH_M,
            ChassisId.WHEEL_DIAMETER_M / 2.0,
            null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) // Deviations standards de mesure
        );
        this.timer.start();
    }

    public Rotation2d robotRotation() {
        return sim.getHeading();
    }

    public double leftMotorPositionM() {
        return sim.getLeftPositionMeters();
    }

    public double rightMotorPositionM() {
        return sim.getRightPositionMeters();
    }

    public DifferentialDriveWheelSpeeds wheelSpeedsMS() {
        return new DifferentialDriveWheelSpeeds(sim.getLeftVelocityMetersPerSecond(), sim.getRightVelocityMetersPerSecond());
    }

    public void simulationPeriodic() {
        // La simulation suppose qu'un voltage positif est pour avancer, donc il faut inverser pour le moteur droit. Mais comme on fait l'inversion au niveau matériel, les voltages retournés par les encodeurs ne sont pas inversés.

        // On "sauve" la transmission courante pour utiliser la même durant toute la durée de la méthode, au cas où elle
        // change pendant.
        var activeGearing = transmission.get();
        sim.setCurrentGearing(activeGearing.getGearingRatioEncoderShaftTurnsOverWheelShaftTurns());

        // On dit à la simulation quels sont les voltages que l'on est en train d'appliquer aux moteurs.
        sim.setInputs(
            leftMotor.get() * RobotController.getInputVoltage(),
            rightMotor.get() * RobotController.getInputVoltage()
        );

        // Combien de temps s'est écoulé depuis la dernière fois
        sim.update(timer.get());
        timer.reset();

        // On écoute la simulation pour savoir quels devraient être les valeurs des capteurs.
        leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        leftMotorSim.setIntegratedSensorRawPosition((int)activeGearing.ticksForDistanceM(sim.getLeftPositionMeters()));
        leftMotorSim.setIntegratedSensorVelocity((int)activeGearing.speedT100MSForSpeedMS(sim.getLeftVelocityMetersPerSecond()));

        rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        rightMotorSim.setIntegratedSensorRawPosition((int)activeGearing.ticksForDistanceM(sim.getRightPositionMeters()));
        rightMotorSim.setIntegratedSensorVelocity((int)activeGearing.speedT100MSForSpeedMS(sim.getRightVelocityMetersPerSecond()));
    }    
}
