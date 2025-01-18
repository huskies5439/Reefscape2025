// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ascenseur extends SubsystemBase {

  private SparkFlexConfig moteurConfig = new SparkFlexConfig();
  private SparkFlex moteur = new SparkFlex(0, null);
  private ProfiledPIDController pidAscenseur = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(0, 0));
  private double conversionEncoder;
  private double forceAscenseur;

  private final DigitalInput limitSwitch = new DigitalInput(0);

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  private double hauteurCible;

  public Ascenseur() {

    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kBrake);
    moteur.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Hauteur Ascenseur", getPosition());
    forceAscenseur = SmartDashboard.getNumber("voltage ascenseur", 0);
    SmartDashboard.putBoolean("At limit Switch", isLimitSwitch());
  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public void monter() {
    setVoltage(1);
  }

  public void descendre() {
    setVoltage(-1);
  }

  public void stop() {
    setVoltage(0);
  }

  public void resetPID() {
    pidAscenseur.reset(getPosition());
  }

  @Logged
  public double getPosition() {
    return moteur.getEncoder().getPosition();
  }

  public void setPID(double cible) {
    double voltagePID = pidAscenseur.calculate(getPosition(), cible);

    double voltageFF = feedforward.calculate(pidAscenseur.getSetpoint().velocity);

    setVoltage(voltagePID + voltageFF);
  }

  public boolean atCible() {
    return pidAscenseur.atGoal();
  }

  public void resetEncoders() {
    moteur.getEncoder().setPosition(0);
  }

  public boolean isLimitSwitch() {
    return !limitSwitch.get();

  }

  public void setHauteurCible(double cible) {
    hauteurCible = cible;

  }

  public double getHauteurCible() {
    return hauteurCible;
  }

}
