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

public class AlgueManip extends SubsystemBase {

  // Créer les moteurs
  private SparkMax moteurDroit = new SparkMax(11, MotorType.kBrushless);
  private SparkMax moteurGauche = new SparkMax(12, MotorType.kBrushless);

  // Créer configs des moteurs
  private SparkMaxConfig moteurDroitConfig = new SparkMaxConfig();
  private SparkMaxConfig moteurGauchceConfig = new SparkMaxConfig();

  // Capteur
  private DigitalInput limitSwitch = new DigitalInput(4);

  public AlgueManip() {
    
    // associe configs moteur droit
    moteurDroitConfig.inverted(false);
    moteurDroitConfig.idleMode(IdleMode.kBrake);
    moteurDroitConfig.smartCurrentLimit(20);//Pour le hold de l'algue
    moteurDroit.configure(moteurDroitConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // associe configs moteur gauche
    moteurGauchceConfig.inverted(true);
    moteurGauchceConfig.idleMode(IdleMode.kBrake);
    moteurGauchceConfig.smartCurrentLimit(20);
    moteurGauche.configure(moteurGauchceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Capteur Algue : ", isAlgue());
  }

  public void setVoltage(double voltage) {
    moteurDroit.setVoltage(voltage);
    moteurGauche.setVoltage(voltage);
  }

  // gober/lancer/stop avec le manip d'algues
  public void gober() {
    setVoltage(2); // Voltages a reverifier
  }

  public void sortir() {
    setVoltage(-4);
  }

  public void stop() {
    setVoltage(0);
  }

  public void hold() {
    setVoltage(0.5);//Nécessite la limite de courant
  }

  // Retourne s'il y a de l'algue dans le manip
  public boolean isAlgue() {
    return !limitSwitch.get();
  }


//Commandes inlines
  public Command goberCommand(){
    return Commands.run(this::gober, this).until(this::isAlgue);
  }

  public Command sortirCommand(){
    return Commands.runEnd(this::sortir,this::stop, this);
  }
}
