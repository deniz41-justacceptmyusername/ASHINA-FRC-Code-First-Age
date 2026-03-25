package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClimbingSubsystem extends SubsystemBase {
    private final SparkMax m_climberRight = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax m_climberLeft = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax m_climberNew = new SparkMax(3, MotorType.kBrushless);
>>>>>>> main

    public ClimbingSubsystem() {
        // Temel konfigürasyon oluşturuldu
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        // Sol motor için ayarlar
        SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
        leftClimberConfig.apply(baseConfig); 
        leftClimberConfig.inverted(false); 

        // Sağ motor için ayarlar
        SparkMaxConfig rightClimberConfig = new SparkMaxConfig();
        rightClimberConfig.apply(baseConfig); 
        rightClimberConfig.inverted(true); 

        // Yeni tırmanma motoru (m_climberNew) için ayarlar
        SparkMaxConfig newClimberConfig = new SparkMaxConfig();
        newClimberConfig.apply(baseConfig);
        newClimberConfig.inverted(false); // Varsayılan yön

        // Konfigürasyonlar SADECE BİR KERE motorlara yazılıyor
        m_climberRight.configure(rightClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climberLeft.configure(leftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climberNew.configure(newClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void getRight(double speed) {
        // Motoru bir yöne döndürmek için pozitif hız

        m_climberNew.set(speed);
    }

    public void getLeft(double speed) {
        SparkMaxConfig m_climbing = new SparkMaxConfig();

        SparkMaxConfig newClimberConfig = new SparkMaxConfig();
        newClimberConfig.apply(m_climbing); // konfigürasyon
        m_climbing.idleMode(SparkMaxConfig.IdleMode.kBrake);
        newClimberConfig.inverted(false);

        m_climberNew.configure(newClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climberNew.set(speed);
    }
    
    public void RunClimber(double speed) {
        // motorları çalıştırır
        m_climberRight.set(speed);
        m_climberLeft.set(speed);
    }

    public void Climbstop() {
        // motorları durdurur
        m_climberRight.stopMotor();
        m_climberLeft.stopMotor();
        m_climberNew.stopMotor(); // Güvenlik için bunu da ekledik
    }
}