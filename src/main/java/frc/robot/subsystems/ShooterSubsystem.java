package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher; // ARRAY OLARAK DEĞİŞTİ
import edu.wpi.first.wpilibj.Timer;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight = new TalonFX(71, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(72, ShooterConstants.kCANBus);

    private Timer shotTimer = new Timer();
    private boolean isBallInAir = false;

    private double startX = 0.0;
    private double startY = 0.0;
    private double startZ = 0.5;

    private double velocity = 10.0;
    private double pitchAngle = Math.toRadians(45);
    private double yawAngle = Math.toRadians(0);

    private final double GRAVITY = 9.81;

    // 1. DÜZELTİLEN KISIM: Veriyi tek bir obje değil, "Toplar Listesi" olarak yayınlıyoruz
    private final StructArrayPublisher<Pose3d> ballPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getStructArrayTopic("HarbiTop", Pose3d.struct).publish();

    private final DutyCycleOut m_request = new DutyCycleOut(0.0);

    double FatihSultanMehmet = 1453;

    public ShooterSubsystem() {
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(currentConfigs);
    }

    public void shootBall(double currentRobotX, double currentRobotY, double currentRobotYaw) {
        this.startX = currentRobotX;
        this.startY = currentRobotY;
        this.yawAngle = currentRobotYaw;

        isBallInAir = true;
        shotTimer.restart();
    }

    public void setShooterSpeed(double speed) {
        m_shooterRight.setControl(m_request.withOutput(speed));
        m_shooterLeft.setControl(m_request.withOutput(speed));
    }

    public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
    }

    // --- YENİ EKLENEN METODLAR BURADAN BAŞLIYOR ---

    /**
     * Hedefe olan mesafeye göre shooter motorlarına verilecek gücü (%0 ile %100 arası) hesaplar.
     * İleride sistemi RPM tabanlı yaparsanız bu kısmı bir Look-up Table'a çevirmelisiniz.
     */
    public double getWantedDutyCycle(double distanceMeters) {
        // Lineer bir artış varsayıyoruz: Uzaklık arttıkça güç artar.
        // Örnek: 2 metrede %50, 5 metrede %80. 
        // DİKKAT: Bu `0.1` ve `0.3` değerlerini sahada atış yaparak kendi robotunuza göre kalibre etmelisiniz.
        double power = (distanceMeters * 0.1) + 0.3; 
        
        // Güvenlik sınırları: Motor gücü %100'ü (1.0) geçemez, %20'nin (0.2) altına inemez.
        if (power > 1.0) power = 1.0; 
        if (power < 0.2) power = 0.2;
        
        return power;
    }

    /**
     * Motorların hedeflenen hıza ulaşıp ulaşmadığını kontrol eder.
     * PID ile RPM kontrolü yapmadığımız için, motora binen voltaj üzerinden tahmini bir "hazır" bilgisi döndürür.
     */
    public boolean isAtSpeed(double wantedDutyCycle) {
        // Phoenix 6 kütüphanesinden motorun anlık voltajını çekiyoruz.
        double currentVoltage = m_shooterRight.getMotorVoltage().getValueAsDouble();
        
        // Hedeflenen voltaj (Robot bataryasını 12V varsayıyoruz)
        double targetVoltage = wantedDutyCycle * 12.0;
        
        // Mevcut voltaj, hedeflenen voltaja yeterince yakınsa (örneğin 1.5V altı bir fark varsa) hazırız demektir.
        return currentVoltage >= (targetVoltage - 1.5);
    }

    // --- YENİ EKLENEN METODLAR BURADA BİTİYOR ---

    @Override
    public void simulationPeriodic() {
        if (isBallInAir) {
            double t = shotTimer.get(); // Havada geçen süre

            // Matematiksel Fizik Hesaplamaları
            double z = startZ + (velocity * Math.sin(pitchAngle) * t) - (0.5 * GRAVITY * Math.pow(t, 2));
            double horizontalDistance = velocity * Math.cos(pitchAngle) * t;
            double x = startX + (horizontalDistance * Math.cos(yawAngle));
            double y = startY + (horizontalDistance * Math.sin(yawAngle));

            if (z < 0.1) { 
                isBallInAir = false;
                shotTimer.stop();
                z = 0.1; 
            }

            // 2. DÜZELTİLEN KISIM: Veriyi dizi (array) içine alıp gönderiyoruz
            Pose3d simulatedBallPose = new Pose3d(x, y, z, new Rotation3d());
            ballPublisher.set(new Pose3d[] { simulatedBallPose });

        } else {
            // 3. DÜZELTİLEN KISIM: Top havada değilken sahanın altına (yine dizi olarak) sakla
            ballPublisher.set(new Pose3d[] { new Pose3d(0.0, 0.0, -5.0, new Rotation3d()) });
        }
    }
}