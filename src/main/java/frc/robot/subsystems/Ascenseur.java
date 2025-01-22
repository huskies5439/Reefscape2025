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

  // moteur + config
  private SparkFlex moteur1 = new SparkFlex(0, null);
  private SparkFlexConfig moteurConfig = new SparkFlexConfig();

  private SparkFlex moteur2 = new SparkFlex(1, null);

  // PID
  private ProfiledPIDController pidAscenseur = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(0, 0));
  private double conversionEncoder;
  private double forceAscenseur;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  // capteur
  private final DigitalInput limitSwitch = new DigitalInput(0);

  private double hauteurCible;

  public Ascenseur() {
    // associe parametres moteurs
    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kBrake);
    // associe les configs aux moteurs
    moteur1.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    moteur2.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Hauteur Ascenseur", getPositionVortex());
    forceAscenseur = SmartDashboard.getNumber("voltage ascenseur", 0);
    SmartDashboard.putBoolean("At limit Switch", isLimitSwitch());
  }

  @Logged
  // Retourne la position de l'encodeur
  public double getPositionVortex() {
    return moteur1.getEncoder().getPosition();
  }

  // reset les encodeurs des vortex
  public void resetEncodersVortex() {
    moteur1.getEncoder().setPosition(0);
    moteur2.getEncoder().setPosition(0);
  }

  // Donne un voltage aux moteurs
  public void setVoltageVortex(double voltage) {
    moteur1.setVoltage(voltage);
    moteur2.setVoltage(voltage);
  }

  // monter/descendre/arrêter l'ascenseur
  public void monter() {
    setVoltageVortex(1);
  }

  public void descendre() {
    setVoltageVortex(-1);
  }

  public void stop() {
    setVoltageVortex(0);
  }

  // PID + FeedForward
  public void setPID(double cible) {
    double voltagePID = pidAscenseur.calculate(getPositionVortex(), cible);

    double voltageFF = feedforward.calculate(pidAscenseur.getSetpoint().velocity);

    setVoltageVortex(voltagePID + voltageFF);
  }

  public void resetPID() {
    pidAscenseur.reset(getPositionVortex());
  }

  // Donne la cible a l'ascenseur
  public void setHauteurCible(double cible) {
    hauteurCible = cible;

  }

  // La hauteur de la cible
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

}
