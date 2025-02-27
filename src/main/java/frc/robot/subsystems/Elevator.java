package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // Two motors for the elevator drive.
  private final TalonFX m_motor1 = new TalonFX(ElevatorConstants.kElevatorMotorID1, "ChooChooTrain");
  private final TalonFX m_motor2 = new TalonFX(ElevatorConstants.kElevatorMotorID2, "ChooChooTrain");
  
  // Gravity compensation constant
  private final double kGravityCompensation = 0.03;
  
  // Add Motion Magic control request
  private int m_printCount = 0;
  private double INITIAL_OFFSET = 0;
  private boolean hasInitialized = false;

  public Elevator() {
    // Configure both motors
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Configure feedback and gear ratio if needed
    config.Feedback.SensorToMechanismRatio = 9.0; // Adjust based on your gear ratio
    
    // Configure Motion Magic parameters
    config.MotionMagic.withMotionMagicCruiseVelocity(10.0)  // Reduced from 20.0
                      .withMotionMagicAcceleration(20.0)    // Reduced from 40.0
                      .withMotionMagicJerk(2000.0);         // Reduced from 4000.0
    
    // Configure PID values
    config.Slot0.kP = 30.0;  // Reduced from 60.0
    config.Slot0.kI = 0.0;   
    config.Slot0.kD = 0.1;   // Reduced from 0.5
    config.Slot0.kS = 0.25;  
    config.Slot0.kV = 1.1;   
    config.Slot0.kA = 0.05;  

    // Apply configuration to both motors
    m_motor1.getConfigurator().apply(config);
    m_motor2.getConfigurator().apply(config);
    
    // Set motor2 to follow motor1 in opposite direction
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true));
  }

  /**
   * Moves the elevator to a specific position using Motion Magic
   * @param targetPosition The target position in rotations
   */
  // public void setPosition(double targetPosition) {
  //   m_motor1.setNeutralMode(NeutralModeValue.Brake);
  //   m_motor2.setNeutralMode(NeutralModeValue.Brake);
  //   m_motor1.setControl(m_motionMagic.withPosition(targetPosition).withSlot(0));
  // }

  // Overloaded method for DynamicMotionMagicVoltage input
  public void setPositionWithRequest(MotionMagicVoltage request) {
    m_motor1.setNeutralMode(NeutralModeValue.Brake);
    m_motor2.setNeutralMode(NeutralModeValue.Brake);
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true));
    m_motor1.setControl(request.withPosition(-request.Position));
  }

  @Override
  public void periodic() {
    // Initialize offset once we get a non-zero reading
    if (!hasInitialized) {
      double currentPosition = m_motor1.getPosition().getValueAsDouble();
      if (currentPosition != 0) {
        INITIAL_OFFSET = currentPosition;
        hasInitialized = true;
        System.out.println("Initialized elevator offset to: " + INITIAL_OFFSET);
      }
    }

    // Print position every 10 cycles
    if (++m_printCount >= 10) {
      m_printCount = 0;
      double position = (m_motor1.getPosition().getValueAsDouble() - INITIAL_OFFSET) * -1;
      System.out.println("Elevator Position: " + position);
      System.out.println("Elevator Velocity: " + m_motor1.getVelocity());
    }
  }

  /**
   * Moves the elevator at the given speed.
   * Positive values move the elevator up and negative values move it down.
   *
   * @param speed A value between -1.0 and 1.0 representing motor output.
   */
  public void moveElevator(double speed) {
    m_motor1.setNeutralMode(NeutralModeValue.Brake);
    m_motor2.setNeutralMode(NeutralModeValue.Brake);
    
    m_motor1.setControl(new DutyCycleOut(-speed * 0.5));
    m_motor2.setControl(new DutyCycleOut(speed * 0.5));
  }

  /** Stops the elevator. */
  public void stop() {
    moveElevator(0);
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

  /**
   * Moves the elevator based on joystick input with gravity compensation.
   *
   * <p>If there’s no input, a constant upward command is applied to counteract gravity.
   * When moving upward, the gravity compensation is added so that the motor provides extra power
   * to overcome gravity, while downward motion is left to gravity.
   *
   * @param joystickInput The raw joystick input value.
   */
  public void moveElevatorWithGravity(double joystickInput) {
    double speed = applyDeadband(joystickInput, 0.05);
    if (speed == 0) {
      // No joystick input: hold position with gravity compensation.
      moveElevator(kGravityCompensation);
    } else if (speed < 0) {
      // Joystick pushed upward (negative value): add gravity compensation.
      moveElevator(-speed + kGravityCompensation);
    } else {
      // Joystick pushed downward (positive value): let gravity assist.
      moveElevator(-speed);
    }
  }
}
