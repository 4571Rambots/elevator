package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator();
  // Create a joystick on port 0 (adjust if needed)
  private final CommandXboxController joysticks = new CommandXboxController(3);

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

    joysticks.b().onTrue(
        new InstantCommand(() -> m_elevator.setPosition(100), m_elevator));

  }
}