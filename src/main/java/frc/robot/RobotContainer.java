package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem();

  private final CommandXboxController controller =
      new CommandXboxController(
          OperatorConstants.kDriverControllerPort);

  public RobotContainer() {

    configureBindings();

    // 🔥 Driver relative input stream
    SwerveInputStream input =
        SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX())
        .withControllerRotationAxis(
            () -> -controller.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // 🔥 Varsayılan sürüş komutu
    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(input));
  }

  private void configureBindings() {

    // Start → ileri yön reset
    controller.start()
        .onTrue(new InstantCommand(
            drivebase::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}