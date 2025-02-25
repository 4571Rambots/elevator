package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Rotations;


import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // Designate one motor as the leader and the other as the follower.
  private final TalonFX m_motorLeader = new TalonFX(ElevatorConstants.kElevatorMotorID1, "ChooChooTrain");
  private final TalonFX m_motorFollower = new TalonFX(ElevatorConstants.kElevatorMotorID2, "ChooChooTrain");

  // Create a PositionVoltage control request for closed-loop position control.
  private final PositionVoltage m_positionControl = new PositionVoltage(0);

  public Elevator() {
    // Set both motors to Brake mode.
    m_motorLeader.setNeutralMode(NeutralModeValue.Brake);
    m_motorFollower.setNeutralMode(NeutralModeValue.Brake);

    // Configure the follower motor to follow the leader.
    m_motorFollower.setControl(new Follower(m_motorLeader.getDeviceID(), false));

    // Configure closed-loop gains for position control on the leader motor.
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.Slot0.kS = 0.24; 
    leaderConfig.Slot0.kV = 0.12;
    leaderConfig.Slot0.kP = 4.8;
    leaderConfig.Slot0.kI = 0.0;
    leaderConfig.Slot0.kD = 0.1;
    m_motorLeader.getConfigurator().apply(leaderConfig, 0.050);
  }

  /**
   * Manually moves the elevator at the given speed.
   * Positive values move the elevator up; negative values move it down.
   * This uses open-loop control.
   *
   * @param speed A value between -1.0 and 1.0 representing motor output.
   */
  public void moveElevator(double speed) {
    m_motorLeader.setControl(new DutyCycleOut(-speed * 0.5));
  }

  /**
   * Moves the elevator to a specific target position (in rotations) using closed-loop control.
   *
   * @param position The target position in rotations.
   */
  public void goToPosition(double position) {
    m_motorLeader.setControl(m_positionControl.withPosition(position));
  }

  /** Stops the elevator by commanding zero output. */
  public void stop() {
    m_motorLeader.setControl(new DutyCycleOut(0));
  }

  /**
   * Returns the current elevator position (in rotations) from the sensor.
   * This allows you to record a target position after manually moving the elevator.
   *
   * @return The current position in rotations.
   */
  public double getCurrentPosition() {
    return m_motorLeader.getRotorPosition().getValue().in(Rotations);
  }
}
