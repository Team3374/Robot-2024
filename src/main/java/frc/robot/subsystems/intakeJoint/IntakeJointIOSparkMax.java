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

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeJointIOSparkMax implements IntakeJointIO {
  private static final double GEAR_RATIO = 48;

  private final CANSparkMax leader = new CANSparkMax(53, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();

  public IntakeJointIOSparkMax() {
    leader.restoreFactoryDefaults();

    leader.setCANTimeout(250);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    leader.setInverted(true);

    encoder.setPosition(0);

    leader.burnFlash();
  }

  @Override
  public void updateInputs(IntakeJointIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  @Override
  public void setSpeed(double speed) {
    leader.set(speed);
  }

  @Override
  public void zeroPosition() {
    encoder.setPosition(0);
  }

  @Override
  public double getRawPosition() {
    return encoder.getPosition();
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
