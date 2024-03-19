// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.Hashtable;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Utils;

// public class SensorSubsystem extends SubsystemBase {
//   private final ArrayList<DigitalInput> m_sensorList = new ArrayList<>();
//   private final Hashtable<DigitalInput, Boolean> m_sensorStates = new Hashtable<>();
//   private final Hashtable<DigitalInput, FunctionalInterface> m

//   public SensorSubsystem() {}

//   public void addAllenBradley(DigitalInput sensor, FunctionalInterface changedCallback) {
//     m_sensorList.add(sensor);
//     m_sensorStates.put(sensor, false);
//   }

//   @Override
//   public void periodic() {
//     for (DigitalInput sensor : m_sensorList) {
//       final boolean previous = m_sensorStates.get(sensor);
//       final boolean current = Utils.isAllenBradleyTripped(sensor);

//       if (previous != current) {

//       }

//       m_sensorStates.put(sensor, current);
//     }
//   }
// }
