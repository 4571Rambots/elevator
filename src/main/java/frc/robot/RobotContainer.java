package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator();
  // Create a CommandXboxController on port 1 (adjust if necessary)
  private final CommandXboxController m_controller = new CommandXboxController(1);

  public RobotContainer() {
    // Default command: use the left stick's Y-axis for manual (open-loop) elevator control.
    m_elevator.setDefaultCommand(new RunCommand(
      () -> {
        double speed = m_controller.getLeftY();
        m_elevator.moveElevator(-speed);
      },
      m_elevator));

    // Bind the A button (using onTrue) to move the elevator to a preset position.
    m_controller.a().onTrue(
      new InstantCommand(
        () -> m_elevator.goToPosition(ElevatorConstants.kElevatorPositionTarget),
        m_elevator
      )
    );
    
    // Bind the B button to capture and print the current position.
    m_controller.b().onTrue(
      new InstantCommand(() -> {
        double currentPosition = m_elevator.getCurrentPosition();
        System.out.println("Captured Elevator Position: " + currentPosition);
        // Optionally, you could save this value somewhere for later use.
      }, m_elevator)
    );
  }
}
