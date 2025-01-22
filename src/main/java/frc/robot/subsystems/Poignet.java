// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Poignet extends SubsystemBase {
  // créé le moteur + sa config
  private SparkFlex moteur = new SparkFlex(13, MotorType.kBrushless);
  private SparkFlexConfig configMoteur = new SparkFlexConfig();
  // créé le capteur
  private DigitalInput capteur = new DigitalInput(4);// Channel a reverifier

  // créé PID + FeedForward
  private ProfiledPIDController pidPoignet = new ProfiledPIDController(0.1, 0, 0,
      new TrapezoidProfile.Constraints(320, 420));

  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

  private double angleCible;

  public Poignet() {
    // associe configs au moteur
    configMoteur.inverted(false);
    configMoteur.idleMode(IdleMode.kBrake);
    moteur.configure(configMoteur, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle Poignet", getPosition());
  }

  // retourne la position poignet ANGLE????
  public double getPosition() {
    return moteur.getEncoder().getPosition();
  }

  public void resetEncoders() {
    moteur.getEncoder().setPosition(0);
  }

  // mets le voltage dans le moteur
  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  /// fonctions de jeu avec le poignet
  public void monter() {
    setVoltage(1);
  }

  public void descendre() {
    setVoltage(-1);
  }

  public void stop() {
    setVoltage(0);
  }

  // PID
  public void setPID(double cible) {
    double voltagePID = pidPoignet.calculate(getPosition(), cible);

    double voltageFF = feedforward.calculate(
        Math.toRadians(getPosition()),
        pidPoignet.getSetpoint().velocity);

    setVoltage(voltagePID + voltageFF);

  }

  public void resetPID() {
    pidPoignet.reset(getPosition());
  }

  public void setAngleCible(double cible) {
    angleCible = cible;
  }

  public double getAngleCible() {
    return angleCible;
  }

  public boolean atCible() {
    return pidPoignet.atGoal();
  }

}
