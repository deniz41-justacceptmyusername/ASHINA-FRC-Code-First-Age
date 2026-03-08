package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootWhileMoving extends Command {
    private final SwerveSubsystem drivebase;
    private final ShooterSubsystem shooter;
    private final CommandXboxController controller;

    // Rotasyonu hedefe kilitlemek için PID Kontrolcü (Değerleri robotta denemen lazım)
    private final PIDController thetaController = new PIDController(0.05, 0, 0);

    // 2026 Reefscape Merkezi (Örnek Koordinat - Metre cinsinden kendi sahana göre ayarla)
    private static final Translation2d TARGET_POS = new Translation2d(8.0, 4.0);
    private static final double SHOOT_ANGLE_RANGE_RAD = Math.toRadians(3.0); // 3 derece hata payı

    public ShootWhileMoving(SwerveSubsystem drivebase, ShooterSubsystem shooter, CommandXboxController controller) {
        this.drivebase = drivebase;
        this.shooter = shooter;
        this.controller = controller;
        
        // Theta controller ayarları (Sürekli dönen bir çember olduğu için)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivebase, shooter);
    }

    @Override
    public void execute() {
        // 1. TAHMİN MATEMATİĞİ (PREDICTION)
        // YAGSL üzerinden anlık saha hızını alıyoruz
        ChassisSpeeds speeds = drivebase.getSwerveDrive().getFieldVelocity();
        Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d robotPositionT0 = drivebase.getPose().getTranslation();

        double linearVelocity = robotVelocity.getNorm();
        
        // İvme sensörümüz olmadığı için sabit bir durma/gecikme süresi öngörüyoruz (0.2 saniye)
        double timeTillStop = 0.2; 

        // Sanal atış pozisyonu (0.2 saniye sonra nerede olacağız?)
        Translation2d robotShootPosition = robotPositionT0.plus(robotVelocity.times(timeTillStop));
        
        // Hedefe olan asıl açı
        Rotation2d targetRotation = TARGET_POS.minus(robotShootPosition).getAngle();
        double distance = robotShootPosition.getDistance(TARGET_POS);

        // 2. SÜRÜŞ KONTROLÜ (Sürücü X-Y'yi kontrol eder, kod rotasyonu kilitler)
        // Joystik değerlerini al (Ölü bölge uygulamasını YAGSL yapıyor ama burada manuel veriyoruz)
        double vX = -controller.getLeftY(); // İleri-Geri
        double vY = -controller.getLeftX(); // Sağ-Sol
        
        // Hedefe dönmek için gereken dönüş hızı (Radyan/saniye)
        double rotationSpeed = thetaController.calculate(
            drivebase.getPose().getRotation().getRadians(), 
            targetRotation.getRadians()
        );

        // YAGSL'ye hareket komutunu gönder
        drivebase.getSwerveDrive().drive(
            new Translation2d(vX, vY).times(frc.robot.Constants.maxSpeed), 
            rotationSpeed, 
            true,  // Saha merkezli sürüş
            false  // Açık döngü
        );

        // 3. SHOOTER KONTROLÜ
        double wantedShooterDutyCycle = shooter.getWantedDutyCycle(distance);
        shooter.setShooterSpeed(wantedShooterDutyCycle);

        double angleError = MathUtil.angleModulus(drivebase.getPose().getRotation().minus(targetRotation).getRadians());
        boolean isAligned = Math.abs(angleError) < SHOOT_ANGLE_RANGE_RAD;
        boolean isAtSpeed = shooter.isAtSpeed(wantedShooterDutyCycle);

        // Eğer açımız doğruysa, hızımız doğruysa ve robot çok hızlı uçmuyorsa ateşle
        if (isAligned && isAtSpeed && linearVelocity < 2.0) {
             // Sadece simülasyon/log için
             shooter.shootBall(robotPositionT0.getX(), robotPositionT0.getY(), drivebase.getPose().getRotation().getRadians());
             
             // NOT: Gerçekte burada topu shooter'a sürecek indexer/intake motorunu çalıştırmalısın.
             // Örnek: intake.getup(); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        drivebase.getSwerveDrive().drive(new Translation2d(0, 0), 0, true, false);
    }
}