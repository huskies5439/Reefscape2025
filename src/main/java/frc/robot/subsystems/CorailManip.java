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

public class CorailManip extends SubsystemBase {

  // Créer moteur + config
  private SparkMax moteur = new SparkMax(13, MotorType.kBrushless);
  private SparkMaxConfig configMoteur = new SparkMaxConfig();

  // LimitSwitch
  private DigitalInput limitSwitch = new DigitalInput(5);

  public CorailManip() {

    // set parametre de config + associe la config au moteur
    configMoteur.inverted(true);
    configMoteur.idleMode(IdleMode.kBrake);
    configMoteur.smartCurrentLimit(5);//Nécessaire pour faire un hold sur le corail
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
    setVoltage(1.5);
  }

  public void sortir() {
    setVoltage(-7);
  }

  public void stop() {
    setVoltage(0);
  }

  public void hold(){
    setVoltage(0.5);
  }

  // retourne s'il y a un corail dans le manip
  public boolean isCorail() {
    return !limitSwitch.get();
  }

//Commandes inlines
   public Command goberCommand(){//La commande équivalente dans le AlgueManip est vraiment plus simple à cause de la commande par défaut.
                                  //Pas mal sûr qu'on pourrait ajuster pour faire la même affaire
    return Commands.run(this::gober,this).until(this::isCorail);
  }

  public Command sortirCommand(){
    return Commands.runEnd(this::sortir, this::stop, this);
  }
}
