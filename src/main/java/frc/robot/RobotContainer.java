package frc.robot;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator();
  // Create a joystick on port 3 (adjust if needed)
  private final CommandXboxController joysticks = new CommandXboxController(3);
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Gravity compensation constant (adjust this value as needed for your elevator)
  private final double kGravityCompensation = 0.03;

  public RobotContainer() {
    // Set the default command for the elevator to continuously read the joystick Y-axis.
    m_elevator.setDefaultCommand(new RunCommand(
        () -> {
          // Read the Y-axis from the joystick and apply deadband
          double rawSpeed = joysticks.getLeftY();
          double speed = applyDeadband(rawSpeed, 0.05);

          // No input: hold position using gravity compensation
          if (speed == 0) {
            m_elevator.moveElevator(kGravityCompensation);
          } 
          // Upward movement (joystick pushed upward typically gives a negative value)
          else if (speed < 0) {
            m_elevator.moveElevator(-speed + kGravityCompensation);
          } 
          // Downward movement: let gravity assist
          else {
            m_elevator.moveElevator(-speed);
          }
        },
        m_elevator));

    // Button bindings to move the elevator to preset positions using Motion Magic
    joysticks.a().whileTrue(
        new RunCommand(
            () -> m_elevator.setPositionWithRequest(m_request.withPosition(1)),
            m_elevator));
    joysticks.b().whileTrue(
        new RunCommand(
            () -> m_elevator.setPositionWithRequest(m_request.withPosition(3)),
            m_elevator));
    joysticks.y().whileTrue(
        new RunCommand(
            () -> m_elevator.setPositionWithRequest(m_request.withPosition(4.5)),
            m_elevator));
  }

  /**
   * Applies a deadband to the joystick value.
   * @param value The raw joystick input.
   * @param deadband The deadband threshold.
   * @return Zero if within the deadband, otherwise returns the original value.
   */
  private double applyDeadband(double value, double deadband) {
    return Math.abs(value) < deadband ? 0 : value;
  }
}
