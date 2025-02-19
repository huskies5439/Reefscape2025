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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CorailManip extends SubsystemBase {

  // Créer moteur + config
  private SparkMax moteur = new SparkMax(13, MotorType.kBrushless);
  private SparkMaxConfig configMoteur = new SparkMaxConfig();

  // InfraRouge
  private DigitalInput lightBreak = new DigitalInput(5);

  public CorailManip() {

    // set parametre de config + associe la config au moteur
    configMoteur.inverted(true);
    configMoteur.idleMode(IdleMode.kCoast);
    moteur.configure(configMoteur, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // SmartDashboard
    // SmartDashboard.putBoolean("Corail dans gobeur", isCorail());
  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  // gober/lancer/stop avec manip de corail
  public void gober() {
    setVoltage(1.5); // Voltages a reverifier
  }

  public void sortir() {
    setVoltage(-2);
  }

  public void stop() {
    setVoltage(0);
  }

  // retourne s'il y a un corail dans le manip
  public boolean isCorail() {
    return !lightBreak.get(); // verifier pour le not !
  }

   public Command goberCommand(){
    return Commands.runEnd(this::gober, this::stop, this).until(this::isCorail).andThen(this::stop, this);
  }

  public Command sortirCommand(){
    return Commands.runEnd(this::sortir, this::stop, this);
  }
}
