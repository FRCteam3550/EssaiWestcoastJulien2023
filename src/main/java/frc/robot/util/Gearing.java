package frc.robot.util;

/**
 * Une classe qui permet de calculer les ratios coches d'encodeur / metres (avancés par le robot, monté par un élévateur, etc...).
 */
public class Gearing {
    private final double encoderTicksPerMeter;
    private final double metersPerEncoderTick;
    private final double gearingRatioEncoderShaftTurnsOverWheelShaftTurns;

    /**
     * Construit un utilitaire de calcul de transmission pour un encodeur.
     * 
     * @param encoderTicksPerRotation
     * @param gearingRatioEncoderShaftTurnsOverWheelShaftTurns La réduction entre l'encodeur et les roues (input sur output). Si l'encodeur est directement sur les roues, donner 1.0.
     * @param wheelDiameterM
     */
    public Gearing(double encoderTicksPerRotation, double gearingRatioEncoderShaftTurnsOverWheelShaftTurns, double wheelDiameterM) {
        this.gearingRatioEncoderShaftTurnsOverWheelShaftTurns = gearingRatioEncoderShaftTurnsOverWheelShaftTurns;
        final var wheelPerimeterM = wheelDiameterM * Math.PI;
        this.metersPerEncoderTick = wheelPerimeterM / encoderTicksPerRotation / gearingRatioEncoderShaftTurnsOverWheelShaftTurns;
        this.encoderTicksPerMeter = 1.0 / metersPerEncoderTick;
    }

    public double getGearingRatioEncoderShaftTurnsOverWheelShaftTurns() {
        return gearingRatioEncoderShaftTurnsOverWheelShaftTurns;
    }
    
    public double ticksForDistanceM(double distanceM) {
        return distanceM * encoderTicksPerMeter;
    }
        
    public double distanceMForTicks(double encoderTicks) {
        return encoderTicks * metersPerEncoderTick;
    }

    public double speedT100MSForSpeedMS(double speedMS) {
        return speedMS * encoderTicksPerMeter * 0.1;
    }
    
    public double speedMSForSpeedT100MS(double speedT100MS) {
        return speedT100MS * 10 * metersPerEncoderTick;
    }
}
