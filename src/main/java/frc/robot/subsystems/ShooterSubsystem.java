package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

private final TalonFX m_shooterRight = new TalonFX(62, ShooterConstants.kCANBus);
private final TalonFX m_shooterLeft = new TalonFX(61, ShooterConstants.kCANBus);
private final PWMSparkMax m_shooternew;


private final DutyCycleOut m_request = new DutyCycleOut(0.0);

public ShooterSubsystem() {
    m_shooternew = new PWMSparkMax(1);
    var currentConfigs = new MotorOutputConfigs();
 currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
}

public void newmotor() {
    var m_shooternew = new MotorOutputConfigs();
m_shooternew.Inverted = InvertedValue.Clockwise_Positive;
}

public void setShooterSpeed(double speed) {
        m_shooterRight.setControl(m_request.withOutput(0.9));
        m_shooterLeft.setControl(m_request.withOutput(-0.9));
        
}
public void newMotor() {
    m_shooternew.set(-0.9);
}

public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
        m_shooternew.stopMotor();
    }

    @Override
    public void periodic() {
        
    }
}