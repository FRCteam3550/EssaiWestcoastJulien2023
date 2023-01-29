package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Robot;

public class Navx {
      /*
     * Lorsque nouvellement initialisé, le navx donne un angle de 0 pendant plusieurs cycles de mesures au lieu de la vraie valeur.
     * Or, il faut la vraie valeur pour initialiser l'odométrie. Je n'ai pas trouvé de méthode m'indiquant que l'initialisation est
     * terminée, donc j'attends jusqu'à ce qu'une valeur différente de 0 apparaisse.
     */
    public static AHRS newReadyNavx() {
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
}
