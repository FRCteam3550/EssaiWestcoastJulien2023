package frc.robot.util;

/**
 * Une classe qui permet de calculer les ratios coches d'encodeur / metres (avancés par le robot, monté par un élévateur, etc...).
 */
public class Transmission {
    private final double cochesParMetre;
    private final double metreParCoche;
    private final double reduction;

    /**
     * Construit un utilitaire de calcul de transmission pour un encodeur.
     * 
     * @param cochesEncodeurParRotation
     * @param reductionTransmission La réduction entre l'encodeur et les roues. Si l'encodeur est directement sur les roues, donner 1.0.
     * @param diametreRoueMetres
     */
    public Transmission(double cochesEncodeurParRotation, double reductionTransmission, double diametreRoueMetres) {
        this.reduction = reductionTransmission;
        final var perimetreRoue = diametreRoueMetres * Math.PI;
        this.metreParCoche = reductionTransmission * perimetreRoue / cochesEncodeurParRotation;
        this.cochesParMetre = 1.0 / metreParCoche;
    }

    public double getReduction() {
        return reduction;
    }
    
    public double cochesPourDistance(double distanceMetres) {
        return distanceMetres * cochesParMetre;
    }
        
    public double distanceMetresPourCoches(double cochesEncodeur) {
        return cochesEncodeur * metreParCoche;
    }

    public double cochesPar100msPourVitesse(double vitesseMetresParSeconde) {
        return vitesseMetresParSeconde * cochesParMetre * 0.1;
    }
    
    public double metresParSecondePourVitesse(double vitesseCochesPar100Ms) {
        return vitesseCochesPar100Ms * 10 * metreParCoche;
    }
}
