// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CorailManip extends SubsystemBase {
  
  //créé moteur + config 
  private SparkMax moteur = new SparkMax(12, MotorType.kBrushless); // ID a reverifier
  private SparkMaxConfig configMoteur = new SparkMaxConfig();

  private DigitalInput capteur = new DigitalInput(3); // Channel a reverifier

  public CorailManip() {
    //associe la config au moteur
    configMoteur.inverted(false);
    configMoteur.idleMode(IdleMode.kCoast);
    moteur.configure(configMoteur, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Corail dans gobeur", isCorail());
  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  ///fonction de jeu avec le manip de corail
  public void gober() {
    setVoltage(4);
  }

  public void lancer() {
    setVoltage(-4);
  }

  public void stop() {
    setVoltage(0);
  }

  //retourne s'il y a un corail
  @Logged
  public boolean isCorail() {
    return !capteur.get();
  }

}
