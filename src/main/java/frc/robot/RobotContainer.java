package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.BaseWestcoast;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  public static final int GAMEPAD_USB_ID = 0;

  // Contrôlleurs
  private final XboxController gamepad = new XboxController(GAMEPAD_USB_ID);
  
  private final JoystickButton xBlueButton = new JoystickButton(gamepad, Button.kX.value);
  private final JoystickButton yYellowButton = new JoystickButton(gamepad, Button.kY.value);
  private final JoystickButton aGreenButton = new JoystickButton(gamepad, Button.kA.value);
  private final JoystickButton rLeftButton = new JoystickButton(gamepad, Button.kLeftBumper.value);
  private final JoystickButton rRightButton = new JoystickButton(gamepad, Button.kRightBumper.value);

  // Sous systèmes
  private final BaseWestcoast driveTrain = new BaseWestcoast(gamepad);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    xBlueButton.onTrue(driveTrain.switchToPrecisionDrive());
    yYellowButton.onTrue(driveTrain.switchToFastDrive());
    aGreenButton.onTrue(driveTrain.goToFrontOfTag0());
    rLeftButton.onTrue(driveTrain.disableCameraEstimate());
    rRightButton.onTrue(driveTrain.enableCameraEstimate());
  }

  public Command getAutonomousCommand() {
    return driveTrain.goToFrontOfTag0();
  }
}
