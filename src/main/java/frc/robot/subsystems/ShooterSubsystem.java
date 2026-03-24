package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight = new TalonFX(62, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(61, ShooterConstants.kCANBus);
    private final PWMSparkMax shooterstopper = new PWMSparkMax(1);
    
    // Hız kontrolü için talep objesi (Velocity cinsinden)
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public ShooterSubsystem() {
    // 1. Konfigürasyon objelerini oluştur
    var shooterConfigs = new MotorOutputConfigs();
    var pidConfigs = new Slot0Configs();

    // 2. Yön ayarlarını yap (Inverted)
    shooterConfigs.Inverted = InvertedValue.Clockwise_Positive; 
    // Diğer motor için ayrı bir config objesi veya sırayla uygulama yapabilirsin
    
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
        // Motorlar belirlediğin RPS değerine ulaşmak için akımı kendisi ayarlar
        m_shooterRight.setControl(m_velocityRequest.withVelocity(-rps));
        m_shooterLeft.setControl(m_velocityRequest.withVelocity(-rps));
    }
    @Override
    public void periodic(){
        double leftVelocity = m_shooterLeft.getVelocity().getValueAsDouble();
        double rightVelocity = m_shooterRight.getVelocity().getValueAsDouble();
        
        if (leftVelocity >= 0.7 && rightVelocity >= 0.7) {
            shooterstopper.set(0.3);
        }
        else {
            shooterstopper.stopMotor();
        }

    }

    public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
    }

    @Override

    public void simulationPeriodic() {

       

    }
    // ... test ve stoptest metodları aynı kalabilir
}