package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; // RunCommand Import edildi
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem; // IntakeSubsystem import edildi
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;

// PathPlanner Kütüphaneleri (Otonom için eklendi)
import com.pathplanner.lib.auto.NamedCommands;
 
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {  
    // Subsystem tanımları
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem(); // Intake buraya bağlandı
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    //private final ClimbingSubsystem m_Climber = new ClimbingSubsystem();
  
    // Xbox Kontrolcüsü (Port 0)
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
    public RobotContainer() {
      // 1. Otonom komutlarını kaydediyoruz
      registerPathPlannerCommands();

      configureBindings();
  
      // Sürüş Giriş Akışı (Input Stream) Yapılandırması
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> m_driverController.getLeftY()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()),
          () -> -m_driverController.getLeftX()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()))
          .withControllerRotationAxis(() -> 
              RobotBase.isSimulation() ? 
              m_driverController.getRightX() * 0.5 : 
              m_driverController.getRightX() * 0.5
          )
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.5+m_driverController.getRightTriggerAxis()*0.5) // Hızı %80'e sınırlar, güvenli sürüş sağlar
          .allianceRelativeControl(true);
  
      // Varsayılan komut olarak sürüşü ata
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
    }

  private void registerPathPlannerCommands() {
    // 1. Intake mekanizmasını indiren komut
    NamedCommands.registerCommand("intake opening", 
        new RunCommand(() -> m_intake.getdown(), m_intake)
            .withTimeout(1.5) 
            .andThen(() -> m_intake.backstop(), m_intake)
    );

    // 2. Sadece içeri alma tekerleklerini döndürür
    NamedCommands.registerCommand("intage begin", 
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.5), m_intake)
            .withTimeout(4.0) 
            .andThen(() -> m_intake.frontstop(), m_intake) 
    );

    // 3. 5 saniyelik atış komutu
    NamedCommands.registerCommand("shooter komutu", 
        new RunCommand(() -> m_shooter.setShooterVelocity(-25.0), m_shooter) 
            .withTimeout(5.0) 
            .andThen(() -> m_shooter.stop(), m_shooter) 
    );
  }

  private void configureBindings() {
    // Start butonu veya B butonu Gyro'yu sıfırlar (Robotun baktığı yer ileri olur)
    m_driverController.start().onTrue(new InstantCommand(drivebase::zeroGyro));
    m_driverController.b().onTrue(new InstantCommand(drivebase::zeroGyro));

    // Tuşa basılı tutunca %70 güçle çalışır, çekince stop() metodunu çağırır
    m_driverController.leftBumper().whileTrue(
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.45), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.frontstop(), m_intake)
    );/* 
    m_driverController.x().whileTrue(
      new RunCommand(() -> m_Climber.RunClimber(0.5),m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbstop(),m_Climber)
    );
    m_driverController.pov(90).whileTrue(
      new RunCommand(() -> m_Climber.getRight(0.5), m_Climber)
    ).onFalse(
      new RunCommand(() -> m_Climber.Climbstop(), m_Climber)
    );
    m_driverController.pov(360).whileTrue(
      new RunCommand(() -> m_Climber.getLeft(0.5), m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbstop(), m_Climber)
    );*/
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
    
// Right Bumper: Shooter'ı belirlenen RPS hızında çalıştır
m_driverController.rightBumper().whileTrue(
    new RunCommand(() -> m_shooter.setShooterVelocity(-70.0), m_shooter)
).onFalse(
    // Tuş bırakıldığında motorları tamamen durdur
    new InstantCommand(() -> m_shooter.stop(), m_shooter)
);

  }

  public Command getAutonomousCommand() {
    // UYARI: Otonom dosyanın PathPlanner arayüzündeki adının "Auto command" olduğundan emin ol.
    return new PathPlannerAuto("Auto command"); 
  }

  // Robot.java'nın drivebase'e ulaşabilmesi için bir köprü görevi görüyor.
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
}