package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem; 
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto; 

public class RobotContainer {  
    // Subsystem tanımları
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  
    // Xbox Kontrolcüsü (Port 0)
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
    public RobotContainer() {
      // 1. PathPlanner komutlarımızı her şeyden önce kaydediyoruz!
      registerPathPlannerCommands();

      configureBindings();
  
      // Sürüş Giriş Akışı (Input Stream) Yapılandırması
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> m_driverController.getLeftY()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()),
          () -> -m_driverController.getLeftX()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()))
          .withControllerRotationAxis(() -> m_driverController.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.5+m_driverController.getRightTriggerAxis()*0.5) 
          .allianceRelativeControl(true);
  
      // Varsayılan komut olarak sürüşü ata
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
    }

  private void registerPathPlannerCommands() {
    // 1. Sadece mekanizmayı indiren komut (Eski haline döndü)
    NamedCommands.registerCommand("intake opening", 
        new RunCommand(() -> m_intake.getdown(), m_intake)
            .withTimeout(1.5) // İnme süresi (ihtiyacına göre değiştir)
            .andThen(() -> m_intake.backstop(), m_intake)
    );

    // 2. YENİ KOMUT: Sadece içeri alma tekerleklerini döndürür
    NamedCommands.registerCommand("intage begin", 
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.5), m_intake)
            .withTimeout(4.0) // 4 saniye boyunca tekerlekler döner
            .andThen(() -> m_intake.frontstop(), m_intake) // Sonra tekerlekleri durdurur
    );

    // 3. 5 saniyelik atış komutu
    NamedCommands.registerCommand("shooter komutu", 
        new RunCommand(() -> m_shooter.setShooterSpeed(0), m_shooter) // UYARI: Ateş etmek için buradaki 0'ı (örn: 0.8) yapmayı unutma!
            .withTimeout(5.0) // Tam 5 saniye boyunca bu komutu çalıştırır
            .andThen(() -> m_shooter.setShooterSpeed(0), m_shooter) // 5 saniye bitince motoru tamamen durdurur
    );
  }

    // BS ile AS rotası arasındaki 5 saniyelik atış komutu:
    NamedCommands.registerCommand("shooter komutu", 
        new RunCommand(() -> m_shooter.setShooterSpeed(0.3), m_shooter) // UYARI: Ateş etmek için buradaki 0'ı (örn: 0.8) yapmayı unutma!
            .withTimeout(5.0) // Tam 5 saniye boyunca bu komutu çalıştırır
            .andThen(() -> m_shooter.setShooterSpeed(0), m_shooter) // 5 saniye bitince motoru tamamen durdurur
    );
  }

  private void configureBindings() {
    m_driverController.start().onTrue(new InstantCommand(drivebase::flipGyro180));
    m_driverController.b().onTrue(new InstantCommand(drivebase::zeroGyro));

    m_driverController.rightBumper().whileTrue(
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.5), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.frontstop(), m_intake)
    );
    m_driverController.pov(0).whileTrue(
      new RunCommand(() -> m_intake.getup(), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.backstop(), m_intake)
    );
        m_driverController.pov(180).whileTrue(
      new RunCommand(() -> m_intake.getdown(), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.backstop(), m_intake)
    );

    m_driverController.rightTrigger().whileTrue(
      new RunCommand(() -> m_shooter.setShooterSpeed(0), m_shooter)
    );
    m_driverController.leftBumper().whileTrue(
      new InstantCommand(() -> {
       Pose2d currentPose = drivebase.getPose();

        m_shooter.shootBall(
          currentPose.getX(),
         currentPose.getY(),
          currentPose.getRotation().getRadians()
        );
      }, m_shooter)
    );
  }

  public Command getAutonomousCommand() {
    // PathPlanner "Autos" sekmesinde oluşturduğun otonom dosyasının ismini buraya yazacaksın
    // Örneğin Auto dosyanın adı "AnaOtonom" ise:
    return new PathPlannerAuto("Auto command"); 
  }

  // Robot.java'nın drivebase'e ulaşabilmesi için bir köprü görevi görüyor.
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
  
  }

