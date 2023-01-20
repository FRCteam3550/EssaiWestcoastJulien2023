package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.BaseWestcoast;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  public static final int MANETTE_NAVIGATION_POSITION_USB = 0;

  // Contrôlleurs
  private final XboxController manetteNavigation = new XboxController(MANETTE_NAVIGATION_POSITION_USB);
  
  private final JoystickButton boutonBleuXNavigation = new JoystickButton(manetteNavigation, Button.kX.value);
  private final JoystickButton boutonJauneYNavigation = new JoystickButton(manetteNavigation, Button.kY.value);
  private final JoystickButton boutonRougeBNavigation = new JoystickButton(manetteNavigation, Button.kB.value);
  private final JoystickButton boutonVertANavigation = new JoystickButton(manetteNavigation, Button.kA.value);

  // Sous systèmes
  private final BaseWestcoast basePilotable = new BaseWestcoast(manetteNavigation);

  public RobotContainer() {
    configureButtonBindings();
    basePilotable.setDefaultCommand(basePilotable.piloter());
  }

  private void configureButtonBindings() {
    boutonBleuXNavigation.onTrue(basePilotable.passeEnModePrecis());
    boutonJauneYNavigation.onTrue(basePilotable.passeEnModeRapide());
    boutonVertANavigation.onTrue(basePilotable.trajectoireAuto());
//    boutonRougeBNavigation.onTrue(basePilotable.commandeRemetAZeroOdometrie());
  }

  public Command getAutonomousCommand() {
    return basePilotable.trajectoireAuto();
  }
}