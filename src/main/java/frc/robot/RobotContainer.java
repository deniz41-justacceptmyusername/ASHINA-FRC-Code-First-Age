package frc.robot;

import java.util.Optional; 
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// 👉 YENİ: Kavisli hesaplama ve ekrana veri yazdırmak için gereken kütüphaneler
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem; 
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {  
    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(m_vision);
    private final IntakeSubsystem m_intake = new IntakeSubsystem(); 
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ClimbingSubsystem m_Climber = new ClimbingSubsystem();
  
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    private final PIDController aimPID = new PIDController(0.03, 0.0, 0.0); 
    private final int TARGET_TAG_27 = 27;
    private final int TARGET_TAG_18 = 18;
    private final int TARGET_TAG_20 = 20;

    // 👉 YENİ: Bütün mesafeler için mükemmel kavis çizen Hız Haritamız!
    private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    
    public RobotContainer() {
      // 🎯 HARİTA DEĞERLERİ (ALAN -> HIZ)
      // Yakınlaştıkça alan büyür (hız düşmeli), uzaklaştıkça alan küçülür (hız artmalı)
      // Ekranda gördüğün değerlere göre burayı istediğin gibi değiştirebilirsin
      shooterSpeedMap.put(0.85, 55.0); // Çok Yakın
      shooterSpeedMap.put(0.70, 65.0); // Yakın 
      shooterSpeedMap.put(0.55, 70.0); // Orta 
      shooterSpeedMap.put(0.35, 78.0); // Uzak
      shooterSpeedMap.put(0.15, 85.0); // Çok Uzak

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

    // Otonom atışını da haritaya bağladım ki otonomda da mesafeye göre atsın!
    NamedCommands.registerCommand("shooter komutu", 
        new RunCommand(() -> {
            Optional<Double> areaOpt = m_vision.getTargetArea(TARGET_TAG_27);
            double currentSpeed = 65.0; 
            if (areaOpt.isPresent()) {
                currentSpeed = shooterSpeedMap.get(areaOpt.get());
            }
            m_shooter.setShooterVelocity(currentSpeed);
        }, m_shooter) 
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
        new RunCommand(() -> m_intake.setIntakeSpeed(-0.475), m_intake)
    ).onFalse(
        new InstantCommand(() -> m_intake.frontstop(), m_intake)
    );
    
    m_driverController.x().whileTrue(
      new RunCommand(() -> m_Climber.RunClimber(0.1),m_Climber)
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
    
    // 👉 YENİ: Harita (Interpolating Map) kullanan Kusursuz Atış Sistemi
    m_driverController.rightBumper().whileTrue(
        new RunCommand(() -> {
            Optional<Double> areaOpt = m_vision.getTargetArea(TARGET_TAG_27);
            double currentSpeed = 60.0; // Kamerayı kapatırsan atacağı varsayılan hız

            if (areaOpt.isPresent()) {
                double alan = areaOpt.get();
                // Haritadan otomatik kavisli hızı çekiyoruz
                currentSpeed = shooterSpeedMap.get(alan);
                
                // Şoför ekranına anlık verileri basıyoruz, buradan bakıp kalibre edebilirsin!
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
            
            // YENİ YAZDIĞIMIZ ÇOKLU TAG METODUNU ÇAĞIRIYORUZ
            var midYawOpt = m_vision.getAverageYaw(20, 26, 27);
            
            // SÜSLÜ PARANTEZ İÇİNDE OLDUĞUMUZ İÇİN "return" KULLANMAK ZORUNLU!
            if (midYawOpt.isPresent()) {
                // Hedefi bulduysa PID ile hesaplanan dönüş hızını "return" et
                return -aimPID.calculate(midYawOpt.get(), 0.0); 
            } else {
                // Göremiyorsa sağ analog çubuğun değerini "return" et (manuel kontrol)
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

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto command"); 
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
}