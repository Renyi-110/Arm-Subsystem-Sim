package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.units.measure.*;

public class ArmSubsystem extends SubsystemBase {
  // Constants
  private final int canID = 1;
  private final double gearRatio = 15;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  private final double maxVelocity = 1; // rad/s
  private final double maxAcceleration = 1; // rad/s²
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;
  private final double armLength = 0.5; // meters
  private final double minAngleDeg = 0;
  private final double maxAngleDeg = 90;

  // Feedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  // Motor control variables
  private final TalonFX motor;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation and visualization related variables
  private final SingleJointedArmSim armSim;
  private final Mechanism2d mech2d = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d root = mech2d.getRoot("ArmRoot", 0.5, 0.1);
  private final MechanismLigament2d armLigament = root.append(new MechanismLigament2d("Arm", armLength, 90));
  
  // Constructor to initialize everything
  public ArmSubsystem() {
    motor = new TalonFX(canID);
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    // Configure motor parameters (PID, current limits, etc.)
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast; // Set brake mode
    config.Feedback.SensorToMechanismRatio = gearRatio; // Set gear ratio for the motor encoder

    motor.getConfigurator().apply(config);
    // Initialize motor position to 0
    motor.setPosition(0);
    
    // Initialize the arm simulation
    armSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      gearRatio,
      SingleJointedArmSim.estimateMOI(armLength, 5),
      armLength,
      Units.degreesToRadians(minAngleDeg),
      Units.degreesToRadians(maxAngleDeg),
      true,
      Units.degreesToRadians(0)
    );

    SmartDashboard.putData("Arm Visualization", mech2d);
  }
  // This method is called periodically to update any information on the SmartDashboard
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
    SmartDashboard.putNumber("Arm/Position (rot)", getPosition());
    SmartDashboard.putNumber("Arm/Velocity (rps)", getVelocity());
    SmartDashboard.putNumber("Arm/Voltage", getVoltage());
    SmartDashboard.putNumber("Arm/Current", getCurrent());
    SmartDashboard.putNumber("Arm/Temperature", getTemperature());
  }

  // This method is called periodically for simulation
  @Override
  public void simulationPeriodic() {
    armSim.setInput(getVoltage());
    armSim.update(0.02);
    armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    armLigament.setLength(armLength);
    
  }
  
  // Method to get current position of the arm (in rotations)
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }


  // Method to get current velocity of the arm (in rotations per second)
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  // Method to get current voltage applied to the motor
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  // Method to get current motor current
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  // Method to get current motor temperature
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

   // Method to convert position to radians (for more precision in calculation)
  public double getPositionRadians() {
    return getPosition() * 2.0 * Math.PI;
  }

  // Method to set the arm angle to a specific value (in degrees)
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  // Method to set the arm angle with specified acceleration (in degrees)
  public void setAngle(double angleDegrees, double acceleration) {
    angleDegrees = Math.max(minAngleDeg, Math.min(maxAngleDeg, angleDegrees));
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);
    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
  }

  // Method to set the arm velocity (in degrees per second)
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

   // Method to set the arm velocity with specified acceleration (in degrees per second)
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    double currentDeg = Units.radiansToDegrees(getPositionRadians());
    if ((currentDeg >= maxAngleDeg && velocityDegPerSec > 0) ||
        (currentDeg <= minAngleDeg && velocityDegPerSec < 0)) {
      velocityDegPerSec = 0;
    }

    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);
    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }

  // Method to set the motor voltage directly
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  // Command to set the arm angle to a specific value
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  // Command to stop the arm by setting velocity to 0
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  // Command to move the arm at a specific velocity
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }
}