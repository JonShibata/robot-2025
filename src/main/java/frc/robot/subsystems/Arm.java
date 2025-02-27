// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX motorArm = new TalonFX(Constants.Arm.motorID);

  private TalonFXSimState motorArmSim;

  private Encoder encoderArm = new Encoder(1, 2);

  private double desiredSpeed = 0;

  private double pidOutput = 0.0;
  private double feedForward = 0.0;
  private double desiredPosition = 0.0;
  private double desiredPositionRateLimited = 0.0;
  private double desiredPositionRateLimitedClamped = 0.0;
  private double gravityFeedForward = Constants.Arm.gravityFeedForward;

  private PIDController controller =
      new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  public Arm() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      motorArmSim = new TalonFXSimState(motorArm);
    }

    motorArm.setPosition(0.0);

    motorArm.setNeutralMode(NeutralModeValue.Brake);
    motorArm.setInverted(true);
  }

  public void setPID(double kp, double ki, double kd) {
    controller.setPID(kp, ki, kd);
  }

  public void setGravityFeedForward(double gravityFeedForward) {
    this.gravityFeedForward = gravityFeedForward;
  }

  public void setDesiredPosition(double position) {
    desiredPosition = position;
  }

  public double calcRateLimit(double desiredPos, double desiredPosPrev, double rateLimit) {
    return MathUtil.clamp(desiredPos, desiredPosPrev - rateLimit, desiredPosPrev + rateLimit);
  }

  public double calcClamp(double desiredPos, double lowLimit, double highLimit) {
    return MathUtil.clamp(desiredPos, lowLimit, highLimit);
  }

  public double calcPID(double currentPos, double desiredPos) {
    return controller.calculate(currentPos, desiredPos);
  }

  public double calcFeedForward(double encoderCounts) {
    return Math.sin((encoderCounts - Constants.Arm.verticalCounts) / Constants.Arm.countsPerRadian)
        * gravityFeedForward;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm-Pos", motorArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Velo", motorArm.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm-Encoder", encoderArm.get());

    desiredPositionRateLimited =
        calcRateLimit(desiredPosition, desiredPositionRateLimited, Constants.Arm.RateLimit);

    desiredPositionRateLimitedClamped =
        calcClamp(
            desiredPositionRateLimited, Constants.Arm.reverseLimit, Constants.Arm.forwardLimit);

    pidOutput = calcPID(encoderArm.get(), desiredPositionRateLimitedClamped);
    feedForward = calcFeedForward(encoderArm.get());
    motorArm.set(pidOutput + feedForward);

    SmartDashboard.putNumber("Arm-FF", feedForward);
    SmartDashboard.putNumber("Arm-PID", pidOutput);
  }

  public void simulationPeriodic() {
    motorArmSim.setSupplyVoltage(
        RobotController
            .getBatteryVoltage()); // need to fix sim capabilities, find talon version of iterate
    // function
  }

  public Command manualArm(DoubleSupplier speed) {
    return this.run(() -> motorArm.set(speed.getAsDouble() * 0.5));
  }

  public void stop() {
    motorArm.set(0);
  }
}
