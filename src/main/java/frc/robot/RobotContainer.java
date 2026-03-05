package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Subsystem tanımı
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // Xbox Kontrolcüsü (Port 0)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    // Sürüş Giriş Akışı (Input Stream) Yapılandırması
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX())
        .withControllerRotationAxis(() -> m_driverController.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8) // Hızı %80'e sınırlar, güvenli sürüş sağlar
        .allianceRelativeControl(true);

    // Varsayılan komut olarak sürüşü ata
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
  }

  private void configureBindings() {
    // Start butonu veya B butonu Gyro'yu sıfırlar (Robotun baktığı yer ileri olur)
    m_driverController.start().onTrue(new InstantCommand(drivebase::zeroGyro));
    m_driverController.b().onTrue(new InstantCommand(drivebase::zeroGyro));
  }

  public Command getAutonomousCommand() {
    // Otonom komutu buraya gelecek
    return null; 
  }

  // 👇 EKLENEN YENİ METOD BURASI 👇
  // Robot.java'nın drivebase'e ulaşabilmesi için bir köprü görevi görüyor.
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
} 