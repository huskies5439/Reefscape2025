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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ascenseur extends SubsystemBase {

  // moteur + config
  private SparkFlex moteur1 = new SparkFlex(9, MotorType.kBrushless);
  private SparkFlex moteur2 = new SparkFlex(10, MotorType.kBrushless);

  private SparkFlexConfig moteurConfig = new SparkFlexConfig();

  // Encodeur
  private Encoder encoder = new Encoder(1, 2); // Channel à reverifier
  private double conversionEncoder;

  // capteur
  private final DigitalInput limitSwitch = new DigitalInput(0);

  // PID
  private ProfiledPIDController pidAscenseur = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(0, 0));

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  // hauteur cible 
  private double hauteurCible;

  public Ascenseur() {
    // set parametres des configs
    moteurConfig.inverted(true);
    moteurConfig.idleMode(IdleMode.kBrake);
    // associe les configs aux moteurs
    moteur1.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    moteur2.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder.setDistancePerPulse(1); // a calculer
  }

  @Override
  public void periodic() {
    // SmartDashboard
    SmartDashboard.putNumber("Hauteur Ascenseur", getPositionVortex()); // Hauteur Ascenceur des VORTEX
    SmartDashboard.putBoolean("At limit Switch", isLimitSwitch());
    SmartDashboard.putNumber("Cible : ", getHauteurCible());
  }

  @Logged
  // Retourne la position de l'encodeur VORTEX
  public double getPositionVortex() {
    return moteur1.getEncoder().getPosition();
  }

  // reset les encodeurs des vortex
  public void resetEncodersVortex() {
    moteur1.getEncoder().setPosition(0);
    moteur2.getEncoder().setPosition(0);
  }

  // Donne un voltage aux moteurs
  public void setVoltage(double voltage) {
    moteur1.setVoltage(voltage);
    moteur2.setVoltage(voltage);
  }

  // monter/descendre/arrêter l'ascenseur
  public void monter() {
    setVoltage(3);
  }

  public void descendre() {
    setVoltage(-1);
  }

  public void stop() {
    setVoltage(0);
  }

  // PID + FeedForward
  public void setPID(double cible) {
    double voltagePID = pidAscenseur.calculate(getPositionVortex(), cible);

    double voltageFF = feedforward.calculate(pidAscenseur.getSetpoint().velocity);

    setVoltage(voltagePID + voltageFF);
  }

  public void resetPID() {
    pidAscenseur.reset(getPositionVortex());
  }

  // cible de l'ascenseur en utilisant la manette operateur
  public void setHauteurCible(double cible) {
    hauteurCible = cible;
  }

  public double getHauteurCible() {
    return hauteurCible;
  }

  public boolean atCible() {
    return pidAscenseur.atGoal();
  }

  // Limit switch
  public boolean isLimitSwitch() {
    return !limitSwitch.get();

  }

  // Encodeur Externe
  public double getPositionExterne() {
    return encoder.getDistance();
  }

  public double getEncodeurSpeed() {
    return encoder.getRate();
  }

  public void resetEncodeurExterne() {
    encoder.reset();
  }
}
