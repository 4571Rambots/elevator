package frc.robot;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator();
  // Create a joystick on port 0 (adjust if needed)
  private final CommandXboxController joysticks = new CommandXboxController(3);
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public RobotContainer() {
    // Set the default command for the elevator to continuously read the joystick
    // Y-axis.
    m_elevator.setDefaultCommand(new RunCommand(
        () -> {
          // Read the Y-axis from the joystick and apply deadband
          double speed = joysticks.getLeftY();
          if (Math.abs(speed) < 0.05) {
            speed = 0;
          }
          // double speed = joysticks.getLeftY();
          m_elevator.moveElevator(-speed);
        },
        m_elevator));

    joysticks.b().whileTrue(
        new RunCommand(
            () -> m_elevator.setPositionWithRequest(m_request.withPosition(3)),
            m_elevator));

  }
}