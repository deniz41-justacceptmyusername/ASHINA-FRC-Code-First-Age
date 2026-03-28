package frc.robot;

import java.util.Optional; 
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d; 

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {  
    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(m_vision);
    private final IntakeSubsystem m_intake = new IntakeSubsystem(); 
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ClimbingSubsystem m_Climber = new ClimbingSubsystem();
    private final LedSubsystem m_led = new LedSubsystem();
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    private final PIDController aimPID = new PIDController(0.03, 0.0, 0.0); 
    private final int TARGET_TAG_27 = 27;

    public RobotContainer() {
      registerPathPlannerCommands();
      configureBindings();
  
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

      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
    }

  private void registerPathPlannerCommands() {
    NamedCommands.registerCommand("intake opening", 
        new RunCommand(() -> m_intake.getdown(), m_intake)
            .withTimeout(1.5) 
            .andThen(() -> m_intake.backstop(), m_intake)
    );

    NamedCommands.registerCommand("intage begin", 
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.5), m_intake)
            .withTimeout(4.0) 
            .andThen(() -> m_intake.frontstop(), m_intake) 
    );
NamedCommands.registerCommand("allign", 
        new RunCommand(() -> {
            // OTONOM İÇİN: Hem Mavi (24, 26, 27) hem Kırmızı (2, 5, 9) ittifak taglerini aynı anda tara!
            var midYawOpt = m_vision.getAverageYaw(21, 26, 28, 2, 5, 10);
            double rotationSpeed = 0.0;
            
            if (midYawOpt.isPresent()) {
                rotationSpeed = -aimPID.calculate(midYawOpt.get(), 0.0); 
            }
            
            drivebase.getSwerveDrive().drive(new Translation2d(0, 0), rotationSpeed, false, false);
        }, drivebase)
        .withTimeout(0.5) 
    );

    NamedCommands.registerCommand("shooter komutu", 
        new RunCommand(() -> m_shooter.setShooterVelocity(65.0), m_shooter) 
            .withTimeout(5.0) 
            .andThen(() -> m_shooter.stop(), m_shooter) 
    );
  }

  private void configureBindings() {
    m_driverController.b().onTrue(new InstantCommand(drivebase::zeroGyro));

    m_driverController.start().onTrue(new InstantCommand(() -> {
        Pose2d mevcutKonum = drivebase.getPose(); 
        Rotation2d tersAci = mevcutKonum.getRotation().plus(Rotation2d.fromDegrees(180));
        drivebase.resetOdometry(new Pose2d(mevcutKonum.getTranslation(), tersAci));
    }));

    m_driverController.leftBumper().whileTrue(
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.54750), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.frontstop(), m_intake)
    );
        m_driverController.y().whileTrue(
      new RunCommand(() -> m_Climber.RunClimber(-0.3),m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbstop(),m_Climber)
    );
    m_driverController.x().whileTrue(
      new RunCommand(() -> m_Climber.RunClimber(0.3),m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbstop(),m_Climber)
    );
    
    m_driverController.pov(90).whileTrue(
      new RunCommand(() -> m_Climber.getRight(0.5), m_Climber)
    ).onFalse(
      new InstantCommand(() -> m_Climber.Climbnewstop(), m_Climber) 
    );

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
        
    m_driverController.povDown().whileTrue(
      new RunCommand(() -> m_intake.getdown(), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.backstop(), m_intake)
    );
    
    // 👉 Right Bumper - Formüle Dayalı Dinamik Atış Sistemi
// 👉 Right Bumper - Formüle Dayalı Dinamik Atış Sistemi (Hem Kırmızı Hem Mavi)
    m_driverController.rightBumper().whileTrue(
        new RunCommand(() -> {
            // Önce Mavi İttifakın ana tagini (26) ara, yoksa Kırmızı İttifakın ana tagini (9) ara
            Optional<Double> areaOpt = m_vision.getTargetArea(26);
            if (areaOpt.isEmpty()) {
                areaOpt = m_vision.getTargetArea(9);
            }

            double currentSpeed = 65.0; // Hedef yoksa atacağı varsayılan hız

            if (areaOpt.isPresent()) {
                double alan = areaOpt.get();
                
                // Hızı doğrudan cebirsel formülümüzden çekiyoruz
                currentSpeed = getDinamikAtisHizi(alan);
                
                // Şoför ekranına anlık verileri basıyoruz, buradan bakıp formülü düzeltebilirsin
                SmartDashboard.putNumber("Kamera Alan Verisi", alan);
                SmartDashboard.putNumber("Dinamik Atis Hizi", currentSpeed);
            }

            m_shooter.setShooterVelocity(currentSpeed); 
        }, m_shooter)
    ).onFalse(
        new InstantCommand(() -> m_shooter.stop(), m_shooter)
    );

SwerveInputStream aimAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()),
        () -> -m_driverController.getLeftX()*(0.5+m_driverController.getRightTriggerAxis()*0.5)*(1.3-m_driverController.getLeftTriggerAxis()))
        .withControllerRotationAxis(() -> {
            
            // Hem Mavi (24, 26, 27) hem Kırmızı (2, 5, 9) ittifak taglerini aynı anda tara!
            // getAverageYaw metodu hangilerini görüyorsa otomatik olarak onların ortalamasını verecektir.
            var midYawOpt = m_vision.getAverageYaw(21, 26, 28, 2, 5, 10);
            
            if (midYawOpt.isPresent()) {
                // Kamera bir hedefe (veya hedeflere) kilitlendi, PID ile oraya dön
                return -aimPID.calculate(midYawOpt.get(), 0.0); 
            } else {
                // Kamera hiçbir şey görmüyorsa şoförün sağ joystiğine kontrolü geri ver
                return m_driverController.getRightX();
            }
            
        })
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.5+m_driverController.getRightTriggerAxis()*0.5)
        .allianceRelativeControl(true);

    m_driverController.a().whileTrue(
        drivebase.driveFieldOriented(aimAngularVelocity)
    );
  }

  // 🎯 MATEMATİKSEL ATIŞ HIZI DENKLEMİ 🎯
  private double getDinamikAtisHizi(double anlikAlan) {
      
      // 👉 İŞTE SENİN FORMÜLÜN BURADA KRAL!
      // Senin dediğin gibi: (Alan * 7^2) 
      // Math.pow(7, 2) demek 7'nin karesi demektir (yani 49).
      // Eğer bu sayılara ekleme çıkarma yapmak istersen denklemi dilediğin gibi uzatabilirsin.
      
      double hesaplananHiz = (Math.sqrt(7.2 / anlikAlan) / 2.0) * 40.0;
      
      // Örnek başka bir denklem denemek istersen yukarıdakini silip şunu yazabilirsin:
      // double hesaplananHiz = (anlikAlan * 50) + 15;

      // GÜVENLİK BARAJI: Yanlış bir hesapta motorlar patlamasın diye hız 30 ile 90 arasına sabitlendi.
      return Math.max(30.0, Math.min(90.0, hesaplananHiz));
  }

  public Command getAutonomousCommand() {
    
    return new PathPlannerAuto("Auto command"); 
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
}