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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ascenseur extends SubsystemBase {

  // moteur + config
  private SparkFlex moteur1 = new SparkFlex(9, MotorType.kBrushless);
  private SparkFlex moteur2 = new SparkFlex(10, MotorType.kBrushless);

  private SparkFlexConfig moteurConfig = new SparkFlexConfig();

  private Servo serrureCage = new Servo(6);

  // Encodeur
  private Encoder encoder = new Encoder(1, 2); // Channel à reverifier

  // capteur
  private final DigitalInput limitSwitch = new DigitalInput(0);

  // PID
  private ProfiledPIDController pidAscenseur = new ProfiledPIDController(100, 0, 0,
      new TrapezoidProfile.Constraints(0.3, 0.1));

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(Constants.kG, 0, 0);

  // hauteur cible
  private double cible;

  private double convertionVortex;

  public Ascenseur() {
    // set parametres des configs
    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kBrake);

    // 14 pignons fait tourner 80. Après, pouli 3/4 de pouce.
    convertionVortex = (14.0 / 80) * Units.inchesToMeters(0.75) * Math.PI;

    moteurConfig.encoder.positionConversionFactor(convertionVortex);
    moteurConfig.encoder.velocityConversionFactor(convertionVortex / 60.0);

    // associe les configs aux moteurs
    moteur1.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    moteur2.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // encodeur a poulie 20 : 68 : 20
    // diamètre poulie 70 mm
    // 1000 conversion mm en m
    // 360 tick par tours
    encoder.setDistancePerPulse((Math.PI * 70.0 / 1000.0) / 360);
  }

  @Override
  public void periodic() {
    // SmartDashboard
    SmartDashboard.putNumber("Vitesse Ascenseur", getVitesseVortex()); // Vitesse Ascenceur
    SmartDashboard.putNumber("Hauteur Ascenseur", getPositionVortex());// Hauteur Ascenceur des Encodeurs
    SmartDashboard.putBoolean("At limit Switch", isLimitSwitch());
    SmartDashboard.putNumber("Cible : ", getCibleRecif());

    if (isLimitSwitch()) {
      resetEncoders();
    }
  }

  // Retourne la position de l'encodeur VORTEX
  public double getPositionVortex() {
    return moteur1.getEncoder().getPosition();
  }

  // reset les encodeurs des vortex
  public void resetEncodersVortex() {
    moteur1.getEncoder().setPosition(0);
    moteur2.getEncoder().setPosition(0);
  }

  public double getVitesseVortex() {
    return moteur1.getEncoder().getVelocity();
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

  public void descendre(double vitesse) {
    setVoltage(vitesse * 1);
  }

  public void stop() {
    setVoltage(0);
  }

  // Servo barrer/debarrer
  public void barrer() {
    serrureCage.setAngle(45);
  }

  public void debarrer() {
    serrureCage.setAngle(135);
  }

  public void resetEncoders() {
    resetEncodersVortex();
    resetEncodeurExterne();
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

  public boolean atCible() {
    return pidAscenseur.atGoal();
  }

  // cible de l'ascenseur en utilisant la manette operateur
  public void setCible(double cible) {
    this.cible = cible;
  }

  public double getCibleRecif() {
    return cible;
  }

  // Limit switch
  public boolean isLimitSwitch() {
    return !limitSwitch.get();

  }

  // Encodeur Externe
  public double getPositionExterne() {
    return encoder.getDistance();
  }

  public double getVitesseExterne() {
    return encoder.getRate();
  }

  public void resetEncodeurExterne() {
    encoder.reset();
  }
}
