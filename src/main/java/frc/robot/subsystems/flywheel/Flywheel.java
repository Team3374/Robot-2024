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

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO ioTop;
  private final FlywheelIO ioBottom;
  private final FlywheelIOInputsAutoLogged inputsTop = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged inputsBottom = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO ioTop, FlywheelIO ioBottom) {
    this.ioTop = ioTop;
    this.ioBottom = ioBottom;


    
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.3, 0.019);
        ioTop.configurePID(0.1, 0.0, 0);
        ioBottom.configurePID(0.0003, 0.0, 0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        ioTop.configurePID(0.5, 0.0, 0.0);
        ioBottom.configurePID(1.0, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    ioTop.updateInputs(inputsTop);
    ioBottom.updateInputs(inputsBottom);

    Logger.processInputs("Flywheel Top", inputsTop);
    Logger.processInputs("Flywheel Bottom", inputsBottom);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    ioTop.setVoltage(volts);
    ioBottom.setVoltage(volts);
  }

  public void runVolts(double voltsTop, double voltsBottom) {
    ioTop.setVoltage(voltsTop);
    ioBottom.setVoltage(voltsBottom);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPMTop, double velocityRPMBottom) {
    var velocityRadPerSecTop = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPMTop);
    ioTop.setVelocity(velocityRadPerSecTop, ffModel.calculate(velocityRadPerSecTop));

    var velocityRadPerSecBottom = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPMBottom);
    ioBottom.setVelocity(velocityRadPerSecBottom, ffModel.calculate(velocityRadPerSecBottom));

    // Log flywheel setpoint
    Logger.recordOutput("FlywheelTop/SetpointRPM", velocityRPMTop);
    Logger.recordOutput("FlywheelBottom/SetpointRPM", velocityRPMBottom);
  }

  /** Stops the flywheel. */
  public void stop() {
    ioTop.stop();
    ioBottom.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getBottomVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputsBottom.velocityRadPerSec);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getTopVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputsTop.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getBottomCharacterizationVelocity() {
    return inputsBottom.velocityRadPerSec;
  }
}
