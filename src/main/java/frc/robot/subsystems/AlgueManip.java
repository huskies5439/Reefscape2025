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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgueManip extends SubsystemBase {

  //créé les moteurs
  private SparkMax moteurDroit = new SparkMax(10, MotorType.kBrushless); // Ids a reverifier
  private SparkMax moteurGauche = new SparkMax(11, MotorType.kBrushless); // Ids a reverifier

  //Configs des moteurs
  private SparkMaxConfig moteurDroitConfig = new SparkMaxConfig();
  private SparkMaxConfig moteurGauchceConfig = new SparkMaxConfig();

  private DigitalInput capteur = new DigitalInput(4);//channel a verifier

  public AlgueManip() {
    // associe configs moteur droit 
    moteurDroitConfig.inverted(true);
    moteurDroitConfig.idleMode(IdleMode.kBrake);
    moteurDroit.configure(moteurDroitConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // associe configs moteur gauche
    moteurGauchceConfig.inverted(false);
    moteurGauchceConfig.idleMode(IdleMode.kBrake);
    moteurGauche.configure(moteurGauchceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

  }

  public void setVoltage(double voltage) {

    moteurDroit.setVoltage(voltage);
    moteurGauche.setVoltage(voltage);

  }

  //gober/lancer/stop avec le manip d'algues
  public void gober() {
    setVoltage(4); // Voltages a reverifier
  }

  public void lancer() {
    setVoltage(-4);
  }

  public void stop() {
    setVoltage(0);
  }

  public boolean isAlgue(){
    return !capteur.get();
  }

}
