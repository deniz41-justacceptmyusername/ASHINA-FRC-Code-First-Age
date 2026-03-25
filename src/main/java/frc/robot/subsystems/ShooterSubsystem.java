package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight = new TalonFX(62, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(61, ShooterConstants.kCANBus);
    private final PWMSparkMax shooterstopper = new PWMSparkMax(1);
    
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    
    private double m_hedefRPS = 0.0;
    private final Timer m_timer = new Timer();
    
    // Sistemin çalışıp çalışmadığını takip eden bayrak
    private boolean m_atisAktif = false; 

    public ShooterSubsystem() {
        var shooterConfigs = new MotorOutputConfigs();
        var pidConfigs = new Slot0Configs();

        shooterConfigs.Inverted = InvertedValue.Clockwise_Positive; 
        pidConfigs.kP = 0.11;
        pidConfigs.kV = 0.12;

        m_shooterRight.getConfigurator().apply(shooterConfigs);
        m_shooterRight.getConfigurator().apply(pidConfigs);

        shooterConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(shooterConfigs);
        m_shooterLeft.getConfigurator().apply(pidConfigs);
    }

    public void setShooterVelocity(double rps) {
        // BURASI DEĞİŞTİ: Artık motorlara gücü hemen burada vermiyoruz.
        // Sadece hedefi belirleyip kronometreyi başlatıyoruz. İşin geri kalanı periodic'te.
        m_hedefRPS = Math.abs(rps); 
        m_atisAktif = true;
        m_timer.restart(); // Tuşa basıldığı an sıfırlanıp başlar
    }

    @Override
    public void periodic() {
        // Eğer bir atış komutu verildiyse
        if (m_atisAktif && m_hedefRPS > 0) {
            double gecenSure = m_timer.get();

            if (gecenSure < 0.1) {
                // 👉 AŞAMA 1: İlk 0.1 saniye
                shooterstopper.set(0.3); // Stopper pozitif dönüyor
                m_shooterRight.setControl(m_velocityRequest.withVelocity(0)); // Ana motorlar bekliyor
                m_shooterLeft.setControl(m_velocityRequest.withVelocity(0));
                
            } else {
                // 👉 AŞAMA 2: 0.1 saniye bitti, ana motorlara ivmeyi ver
                m_shooterRight.setControl(m_velocityRequest.withVelocity(-m_hedefRPS));
                m_shooterLeft.setControl(m_velocityRequest.withVelocity(-m_hedefRPS));

                // Ana motorların anlık hızlarını kontrol et
                double leftVelocity = Math.abs(m_shooterLeft.getVelocity().getValueAsDouble());
                double rightVelocity = Math.abs(m_shooterRight.getVelocity().getValueAsDouble());
                double atisBaraji = m_hedefRPS - 2.0;

                if (leftVelocity >= atisBaraji && rightVelocity >= atisBaraji) {
                    // 👉 AŞAMA 3: Motorlar ivmesini aldı (hedefe ulaştı)
                    shooterstopper.set(-0.3); // Stopper'ı tersine (negatif) çevir ve fırlat!
                } else {
                    // (Aşama 2 Devamı): Hala ivmelenme aşamasındalar, hedefe ulaşmadılar
                    shooterstopper.set(0.3); // Stopper pozitif dönmeye devam ediyor
                }
            }
        } else {
            // Atış komutu yoksa motorları güvenli bir şekilde kapat
            shooterstopper.stopMotor();
            // Control modunu 0 hıza çekiyoruz ki motor kendini frenlesin
            m_shooterRight.setControl(m_velocityRequest.withVelocity(0));
            m_shooterLeft.setControl(m_velocityRequest.withVelocity(0));
        }
    }

    public void stop() {
        m_hedefRPS = 0.0; 
        m_atisAktif = false; // Sistemi inaktif yap
        m_timer.stop();
        m_timer.reset();
        
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
        shooterstopper.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
    }
}