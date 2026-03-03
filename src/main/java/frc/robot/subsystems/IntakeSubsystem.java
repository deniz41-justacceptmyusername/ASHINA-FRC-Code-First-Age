// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_intakeLeader = new TalonFX(51, kCANBus);
  final TalonFX m_intakeFollower = new TalonFX(52, IntakeConstants.kCANBus);
  final TalonFX m_shooter_Leader = new TalonFX(61, IntakeConstants.kCANBus);
  final TalonFX m_shooterFollower = new TalonFX(62, IntakeConstants.kCANBus);

  public IntakeSubsystem() {
        // Follower motor, Leader motoru takip etsin
        m_intakeFollower.setControl(new Follower(m_intakeLeader.getDeviceID()));
    }
  
  public void setIntakeSpeed(double speed) {

  

        m_intakeLeader.setControl(new DutyCycleOut(speed));
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
