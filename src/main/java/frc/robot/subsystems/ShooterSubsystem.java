package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer; // 👇 YENİ: Timer kütüphanesi eklendi
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight = new TalonFX(62, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(61, ShooterConstants.kCANBus);
    private final PWMSparkMax shooterstopper = new PWMSparkMax(1);
    
    // Hız kontrolü için talep objesi (Velocity cinsinden)
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    
    // Anlık hedef hızımızı hafızada tutacak değişken
    private double m_hedefRPS = 0.0;

    // 👇 YENİ: Stopper'ın zamanlamasını takip etmek için gereken değişkenler 👇
    private final Timer m_stopperTimer = new Timer();
    private boolean m_atisBasladi = false; // Sayacın sadece bir kere başlaması için kontrol

    public ShooterSubsystem() {
        // 1. Konfigürasyon objelerini oluştur
        var shooterConfigs = new MotorOutputConfigs();
        var pidConfigs = new Slot0Configs();

        // 2. Yön ayarlarını yap (Inverted)
        shooterConfigs.Inverted = InvertedValue.Clockwise_Positive; 
        
        // 3. PID ve FF değerlerini tanımla
        pidConfigs.kP = 0.11;
        pidConfigs.kV = 0.12;

        // 4. Configurator ile ayarları motorlara gönder
        m_shooterRight.getConfigurator().apply(shooterConfigs);
        m_shooterRight.getConfigurator().apply(pidConfigs);

        // Sol motor için yönü tersine çevirip uygula
        shooterConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(shooterConfigs);
        m_shooterLeft.getConfigurator().apply(pidConfigs);
    }

    /**
     * @param rps Saniyedeki devir sayısı (Rotations Per Second)
     */
    public void setShooterVelocity(double rps) {
        // Hedefi hafızaya al. Negatif sayı gelsen bile pozitif olarak kaydederiz (güvenlik için)
        m_hedefRPS = Math.abs(rps); 

        // Motorlar belirlediğin RPS değerine ulaşmak için akımı kendisi ayarlar
        m_shooterRight.setControl(m_velocityRequest.withVelocity(-rps));
        m_shooterLeft.setControl(m_velocityRequest.withVelocity(-rps));
    }

    @Override
    public void periodic() {
        // Motorlardan anlık hızları al (eksiye dönüyorlarsa diye Math.abs ile pozitife çeviriyoruz)
        double leftVelocity = Math.abs(m_shooterLeft.getVelocity().getValueAsDouble());
        double rightVelocity = Math.abs(m_shooterRight.getVelocity().getValueAsDouble());
        
        // Eğer motorlara verilmiş bir hedef varsa (çalışıyorlarsa)       
        if (m_hedefRPS > 0) {
            double atisBaraji = m_hedefRPS - 2.0; 

            // İki motor da barajı aştıysa sekansı başlat!
            if (leftVelocity >= atisBaraji && rightVelocity >= atisBaraji) {
                
                // Hıza ilk ulaştığımız an sayacı başlatıyoruz
                if (!m_atisBasladi) {
                    m_stopperTimer.restart(); 
                    m_atisBasladi = true;
                }


                if (m_stopperTimer.get() < 0.1) {
                    shooterstopper.set(0.3); 
                } else {
                    shooterstopper.set(-0.3); 
                }

            } else {
                
                m_atisBasladi = false;
                m_stopperTimer.stop();
                shooterstopper.stopMotor(); 
            }
        } else {
            
            m_atisBasladi = false;
            m_stopperTimer.stop();
            shooterstopper.stopMotor();
        }
    }

    public void stop() {
        m_hedefRPS = 0.0; 
        
        m_atisBasladi = false;
        m_stopperTimer.stop();
        m_stopperTimer.reset();
        
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
        shooterstopper.stopMotor();
    }

    @Override
    public void simulationPeriodic() {

    }
}