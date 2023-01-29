package frc.robot.subsystems.westcoast;

public class ChassisId {
  public static final double TRACK_WIDTH_M = 0.77993;
  public static final double WHEEL_DIAMETER_M = 0.1017; // Circonférence: 0.3195

  // Voir résultats caractérisation: https://docs.google.com/spreadsheets/d/1WEG3WhIzJu3VFnJeH4EQ7ALOVoR8XWWcQZ5B1EWVP3w/edit?usp=sharing
  public static final double K_S_LIN_V = 0.24; // Mesurée: 0.24
  public static final double K_V_LIN_VSM = 1.65;
  public static final double K_A_LIN_VS2M = 0.45;  

  public static final double K_V_ROT_VSM = 1.6;
  public static final double K_A_ROT_VS2M = 0.8;  
}
