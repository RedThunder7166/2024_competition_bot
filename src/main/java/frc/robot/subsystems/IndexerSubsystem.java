// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;



public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(IndexerConstants.MOTOR_ID);
  private final VelocityDutyCycle m_request = new VelocityDutyCycle(IndexerConstants.TARGET_VELOCITY);

  private final LauncherSubsystem m_launcher;
  private boolean m_forwardState = false;
  private boolean m_reverseState = false;
  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("IndexerMotor");

  public IndexerSubsystem(LauncherSubsystem launcher) {

    m_launcher = launcher;
    m_shuffleBoardTab.addDouble("MotorTemp", () -> m_motor.getDeviceTemp().getValueAsDouble());

    TalonFXConfiguration config = new TalonFXConfiguration();
    // clockwise is true
    // counterclockwise is false
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    m_motor.getConfigurator().apply(config);
  }

  private static final double speed = 0.65;
  private final DutyCycleOut m_forwardControl = new DutyCycleOut(speed);
  private final DutyCycleOut m_reverseControl = new DutyCycleOut(-speed);

  @Override
  public void periodic() {
    // if (m_launcher.getIsAtLoadingPosition() && m_launcher.getWantsToLoad()) {
    //   m_motor.setControl(m_request);
    // } else {
    //   m_motor.disable();
    // }

    if (m_forwardState) {
      m_motor.setControl(m_forwardControl);
    } else if (m_reverseState) {
      m_motor.setControl(m_reverseControl);
    } else {
      m_motor.disable();
    }
  }

  // public void manualRunDeleteMe(double value) {
  //   m_motor.setControl(new DutyCycleOut(value));
  // }
  // public void manualStopDeleteMe() {
  //   m_motor.disable();
  // }

  public void disabledInit() {
    m_forwardState = false;
    m_reverseState = false;
  }

  // public InstantCommand m_enableForwardCommand = new InstantCommand(() -> {
  //   m_forwardState = true;
  // }, this);
  // public InstantCommand m_disableForwardCommand = new InstantCommand(() -> {
  //   m_forwardState = false;
  // }, this);

  // public InstantCommand m_enableReverseCommand = new InstantCommand(() -> {
  //   m_reverseState = true;
  // }, this);
  // public InstantCommand m_disableReverseCommand = new InstantCommand(() -> {
  //   m_reverseState = false;
  // }, this);

  public void enableForward() {
    m_forwardState = true;
  }
  public void disableForward() {
    m_forwardState = false;
  }

  public void enableReverse() {
    m_reverseState = true;
  }
  public void disableReverse() {
    m_reverseState = false;
  }
}