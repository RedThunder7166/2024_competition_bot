// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_leftClimbMotor = new TalonFX(ClimberConstants.LEFT_CLIMB_MOTOR_ID);
  private final TalonFX m_rightClimbMotor = new TalonFX(ClimberConstants.RIGHT_CLIMB_MOTOR_ID);
  // private final DigitalInput m_rightArmInput = new DigitalInput(ClimberConstants.RIGHT_ARM_LIMIT_SWITCH_ID);
  // private final DigitalInput m_leftArmInput = new DigitalInput(ClimberConstants.LEFT_ARM_LIMIT_SWITCH_ID);

  private DoubleSupplier m_manual_left_supplier;
  private DoubleSupplier m_manual_right_supplier;

  private boolean m_checksEnabled = false;

  private final DutyCycleOut m_leftRequest = new DutyCycleOut(0);
  private final DutyCycleOut m_rightRequest = new DutyCycleOut(0);

  private final PositionDutyCycle m_positionRequest = new PositionDutyCycle(0);

  private static enum Position {
    Low(0, 0),
    High(ClimberConstants.LEFT_TOP_POSITION, ClimberConstants.RIGHT_TOP_POSITION);

    public final double left_position;
    public final double right_position;

    Position(double left_position, double right_position) {
      this.left_position = left_position;
      this.right_position = right_position;
    }
  }
  private Position m_position = Position.Low;
  private boolean m_positionBased = false;

  public ClimberSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // clockwise is true
    // counterclockwise is false
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.DutyCycleNeutralDeadband = ClimberConstants.MANUAL_DEADBAND;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 10;

    config.Slot0.kP = 0.01;

    m_leftClimbMotor.getConfigurator().apply(config);
    m_rightClimbMotor.getConfigurator().apply(config);

    m_leftClimbMotor.setInverted(false);
    m_rightClimbMotor.setInverted(true);

    m_leftClimbMotor.setPosition(0);
    m_rightClimbMotor.setPosition(0);
    
  }

  public void positionLow() {
    enablePositionBased();
    m_position = Position.Low;
  }
  public void positionHigh() {
    enablePositionBased();
    m_position = Position.High;
  }

  public void enablePositionBased() {
    m_positionBased = true;
  }
  public void disablePositionBased() {
    m_positionBased = false;
  }

  public void enableChecks() {
    m_checksEnabled = true;
  }
  public void disableChecks() {
    m_checksEnabled = false;
  }

  public void configureLeftManualMode(DoubleSupplier supplier) {
    m_manual_left_supplier = supplier;
  }
  public void configureRightManualMode(DoubleSupplier supplier) {
    m_manual_right_supplier = supplier;
  }

  private void driveArms(double left_output, double right_output) {
    // if (m_checksEnabled && left_output < 0 && m_leftClimbMotor.getPosition().getValueAsDouble() <= 0) {
    //   m_leftClimbMotor.disable();
    // } else {
      m_leftClimbMotor.setControl(m_leftRequest.withOutput(left_output));
    // }
    
    // if (m_checksEnabled && right_output < 0 && m_rightClimbMotor.getPosition().getValueAsDouble() <= 0) {
    //   m_rightClimbMotor.disable();
    // } else {
      m_rightClimbMotor.setControl(m_rightRequest.withOutput(right_output));
    // }
  }

  @Override
  public void periodic() {
    // if (m_positionBased) {
    //   m_leftClimbMotor.setControl(m_positionRequest.withPosition(m_position.left_position));
    //   m_rightClimbMotor.setControl(m_positionRequest.withPosition(m_position.right_position));
    // } else {
      double left_output = m_manual_left_supplier.getAsDouble();
      double right_output = m_manual_right_supplier.getAsDouble();

      driveArms(left_output, right_output);
    // }
  }
}