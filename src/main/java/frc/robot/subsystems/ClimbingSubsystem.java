package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClimbingSubsystem extends SubsystemBase{
    private final SparkMax m_climberRight = new SparkMax(61, MotorType.kBrushless);
    private final SparkMax m_climberLeft = new SparkMax(62, MotorType.kBrushless);
    private final SparkMax m_climberNew = new SparkMax(63, MotorType.kBrushless);

    public ClimbingSubsystem() {

        SparkMaxConfig m_climbing = new SparkMaxConfig();

        SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
        leftClimberConfig.apply(m_climbing); // konfigürasyon
        m_climbing.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftClimberConfig.inverted(false); // motoru sola doğru çevirir

        SparkMaxConfig rightClimberConfig = new SparkMaxConfig();
        rightClimberConfig.apply(m_climbing); // konfigürasyon
        m_climbing.idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightClimberConfig.inverted(true); // motoru sağa doğru çevirir



        m_climberRight.configure(m_climbing, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // bunların ne işe yaradığını bende anlamadım ama gemini abi böyle yap dedi
        m_climberLeft.configure(leftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void getRight(double speed) {
        SparkMaxConfig m_climbing = new SparkMaxConfig();
        
        SparkMaxConfig newClimberConfig = new SparkMaxConfig();
        newClimberConfig.apply(m_climbing); // konfigürasyon
        m_climbing.idleMode(SparkMaxConfig.IdleMode.kBrake);
        newClimberConfig.inverted(true);

        m_climberNew.configure(newClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    }
}
