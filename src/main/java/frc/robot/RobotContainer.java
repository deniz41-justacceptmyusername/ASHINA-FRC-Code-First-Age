package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem; // IntakeSubsystem import edildi
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.controller.PIDController;

// PathPlanner Kütüphaneleri (Otonom için eklendi)
import com.pathplanner.lib.auto.NamedCommands;
 
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {  
    // Subsystem tanımları
    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(m_vision);
    private final IntakeSubsystem m_intake = new IntakeSubsystem(); 
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ClimbingSubsystem m_Climber = new ClimbingSubsystem();
  
    // Xbox Kontrolcüsü (Port 0)
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    // Otomatik hizalanma için PID Kontrolcüsü ve Hedef Tag ID'si
    private final PIDController aimPID = new PIDController(0.03, 0.0, 0.0); 
    private final int TARGET_TAG_ID = 27; // Örnek: Mavi İttifak Hoparlörü
    
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
          .scaleTranslation(0.5+m_driverController.getRightTriggerAxis()*0.5) 
          .allianceRelativeControl(true);

      // Varsayılan komut normal sürüş
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

      // Tuş atamalarını en son çağırıyoruz
      configureBindings();
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
        new RunCommand(() -> m_shooter.setShooterVelocity(-63.0), m_shooter) 
            .withTimeout(5.0) 
            .andThen(() -> m_shooter.stop(), m_shooter) 
    );
  }

  private void configureBindings() {
    // Start butonu veya B butonu Gyro'yu sıfırlar
    m_driverController.start().onTrue(new InstantCommand(drivebase::zeroGyro));
    m_driverController.b().onTrue(new InstantCommand(drivebase::zeroGyro));

    // Tuşa basılı tutunca %70 güçle çalışır, çekince stop() metodunu çağırır
    m_driverController.leftBumper().whileTrue(
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.475), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.frontstop(), m_intake)
    );
    m_driverController.x().whileTrue(
      new RunCommand(() -> m_Climber.RunClimber(0.5),m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbstop(),m_Climber)
    );
// SAĞA TIRMANMA (90)
    m_driverController.pov(90).whileTrue(
      new RunCommand(() -> m_Climber.getRight(0.5), m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbnewstop(), m_Climber) // RunCommand yerine InstantCommand yapıldı
    );

    // SOLA TIRMANMA (270)
    m_driverController.pov(270).whileTrue( 
      new RunCommand(() -> m_Climber.getLeft(0.5), m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbnewstop(), m_Climber)
    );

    m_driverController.pov(0).whileTrue(
      new RunCommand(() -> m_intake.getup(), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.backstop(), m_intake)
    );
        
    m_driverController.povDown().whileTrue( // pov(180)
      new RunCommand(() -> m_intake.getdown(), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.backstop(), m_intake)
    );
    
// Right Bumper: Shooter'ı belirlenen RPS hızında çalıştır
m_driverController.rightBumper().whileTrue(
    new RunCommand(() -> m_shooter.setShooterVelocity(-75.0), m_shooter)
).onFalse(
    // Tuş bırakıldığında motorları tamamen durdur
    new InstantCommand(() -> m_shooter.stop(), m_shooter)
);


    // 👇 YENİ EKLENEN KISIM: A TUŞUNA BASILI TUTUNCA KİLİTLENME 👇
    // 1. Kilitlenme (Aim) Akışını yaratıyoruz
    SwerveInputStream aimAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()),
        () -> -m_driverController.getLeftX()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()))
        .withControllerRotationAxis(() -> {
            var yawOpt = m_vision.getTargetYaw(TARGET_TAG_ID);
            if (yawOpt.isPresent()) {
                return -aimPID.calculate(yawOpt.get(), 0.0); 
            } else {
                return m_driverController.getRightX();
            }
        })
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.5+m_driverController.getRightTriggerAxis()*0.5)
        .allianceRelativeControl(true);

    // 2. Bu akışı 'A' tuşuna bağlıyoruz
    m_driverController.a().whileTrue(
        drivebase.driveFieldOriented(aimAngularVelocity)
    );
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto command"); 
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
}