// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmPIDTest extends Command {
  private ArmFeedforward feedforward;
  private final Arm arm;
  private double feedForward;

  /** Creates a new ArmPIDTest. */
  public ArmPIDTest(Arm arm) {

    addRequirements(arm);
    this.arm = arm;
    // feedforward = new ArmFeedforward(0, 0, 0);

    SmartDashboard.putNumber("Arm-P", Constants.Arm.kp);
    SmartDashboard.putNumber("Arm-I", Constants.Arm.ki);
    SmartDashboard.putNumber("Arm-D", Constants.Arm.kd);
    SmartDashboard.putNumber("Arm-FF", Constants.Arm.gravityFeedForward);
    SmartDashboard.putNumber("Arm-Des-Pos", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.setPID(
        SmartDashboard.getNumber("Arm-P", 0),
        SmartDashboard.getNumber("Arm-I", 0),
        SmartDashboard.getNumber("Arm-D", 0));

    arm.setGravityFeedForward(SmartDashboard.getNumber("Arm-FF", 0));
    arm.setDesiredPosition(SmartDashboard.getNumber("Arm-Des-Pos", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
