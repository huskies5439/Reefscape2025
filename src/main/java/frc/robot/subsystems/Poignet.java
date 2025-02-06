// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PrivateKey;

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

  // moteur + config
  private SparkFlex moteur = new SparkFlex(14, MotorType.kBrushless);
  private SparkFlexConfig moteurConfig = new SparkFlexConfig();
  private double conversionEncodeur;

  // capteur
  private DigitalInput limitSwitch = new DigitalInput(7);

  // PID + FeedForward
  private ProfiledPIDController pidPoignet = new ProfiledPIDController(0.1, 0, 0,
      new TrapezoidProfile.Constraints(320, 420));

  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

  // hauteur cible de la manette operateur
  private double cibleManetteOperateur;

  public Poignet() {
    // set parametres des configs
    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kBrake);

    // gearbox 4 pour 1 , 9 pour 1
    // 360 degrés par tour
    conversionEncodeur = (1 / 4.0) * (1 / 9.0) * 360;

    moteurConfig.encoder.positionConversionFactor(conversionEncodeur);
    moteurConfig.encoder.velocityConversionFactor(conversionEncodeur / 60.0);

    moteur.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // SmartDashboard
    SmartDashboard.putNumber("Angle Poignet", getAngle());
    SmartDashboard.putNumber("Vitesse Poignet", getVitesse());
    SmartDashboard.putNumber("Cible Poignet : ", cibleManetteOperateur);
  }

  ////////////////// MOTEUR
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

  ///////////////// ENCODEUR

  public double getAngle() {
    return moteur.getEncoder().getPosition();
  }

  public double getVitesse() {
    return moteur.getEncoder().getVelocity();
  }

  public void resetEncoders() {
    moteur.getEncoder().setPosition(0);
  }

  //////////////////// PID + feedForward
  public void setPID(double cible) {
    double voltagePID = pidPoignet.calculate(getAngle(), cible);

    double voltageFF = feedforward.calculate(
        Math.toRadians(getAngle()),
        pidPoignet.getSetpoint().velocity);

    setVoltage(voltagePID + voltageFF);

  }

  public void resetPID() {
    pidPoignet.reset(getAngle());
  }

  public boolean atCible() {
    return pidPoignet.atGoal();
  }

  ////////////////// angles Cible

  public void setCibleManetteOperateur(double cible) {
    this.cibleManetteOperateur = cible;
  }

  public double getCibleManetteOperateur() {
    return cibleManetteOperateur;
  }

  //////////////////// LimitSwitch

  public boolean isHome() {
    return !limitSwitch.get(); // verifier pour le not !
  }

}
