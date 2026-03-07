package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

private final TalonFX m_shooterRight = new TalonFX(71, ShooterConstants.kCANBus);
private final TalonFX m_shooterLeft = new TalonFX(72, ShooterConstants.kCANBus);

private final DutyCycleOut m_request = new DutyCycleOut(0.0);

public ShooterSubsystem() {
    var currentConfigs = new MotorOutputConfigs();
 currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(currentConfigs);
}

public void setShooterSpeed(double speed) {
        m_shooterRight.setControl(m_request.withOutput(speed));
        m_shooterLeft.setControl(m_request.withOutput(speed));
}
public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
  }

}
