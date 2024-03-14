// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.ampJoint;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class AmpJoint extends ProfiledPIDSubsystem {
  private final AmpJointIO io;
  private final AmpJointIOInputsAutoLogged inputs = new AmpJointIOInputsAutoLogged();

  /** Creates a new Amp Joint. */
  public AmpJoint(AmpJointIO io) {
    super(new ProfiledPIDController(.1, 0, 0, new TrapezoidProfile.Constraints(100, 100)), 0);
    this.io = io;

    io.zeroPosition();

    this.enable();
  }

  public void init() {
    io.zeroPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Amp Position", getMeasurement());
    SmartDashboard.putNumber("Target Amp Position", this.getController().getGoal().position);
    super.periodic();

    io.updateInputs(inputs);
  }

  /** Run open loop at the specified voltage. */
  /** Run closed loop at to the specified position. */
  public void runPosition(double positionRad) {
    this.setGoal(positionRad);
  }

  /** Stops the Amp Joint. */
  public void stop() {
    io.setSpeed(0);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    io.setSpeed(output);
  }

  @Override
  protected double getMeasurement() {
    return io.getRawPosition();
  }
}
