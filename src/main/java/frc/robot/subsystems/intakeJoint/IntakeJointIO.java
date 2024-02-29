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

package frc.robot.subsystems.intakeJoint;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeJointIO {
  @AutoLog
  public static class IntakeJointIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeJointIOInputs inputs) {}

  /** Set Speed for Profiled PID */
  public default void setSpeed(double speed) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop to the specified position. */
  public default void setPosition(double positionRad, double ffVolts) {}

  /** Zero Position */
  public default void zeroPosition() {}
  ;

  /** Get raw position */
  public default double getRawPosition() {
    return 0;
  }

  /** Stop in open loop. */
  public default void stop() {}
}
