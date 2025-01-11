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


import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class AlgueManip extends SubsystemBase {
  /** Creates a new AlgueManip. */
  private final SparkMax moteurDroit = new SparkMax(10,MotorType.kBrushless); // Ids a reverifier
  private final SparkMax moteurGauche = new SparkMax(11,MotorType.kBrushless); // Ids a reverifier
  

  public static final SparkMaxConfig moteurDroitConfig = new SparkMaxConfig();
  public static final SparkMaxConfig moteurGauchceConfig = new SparkMaxConfig(); 
  public AlgueManip() {
    // configs moteur droit + gauche
   moteurDroitConfig.inverted(true);
   moteurDroitConfig.idleMode(IdleMode.kBrake);
   moteurDroit.configure(moteurDroitConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   moteurGauchceConfig.inverted(false); 
   moteurGauchceConfig.idleMode(IdleMode.kBrake);
   moteurGauche.configure(moteurGauchceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }



  public void setVoltage(double voltage){

    moteurDroit.setVoltage(voltage);
    moteurGauche.setVoltage(voltage);

  }

  public void gober(){

    setVoltage(4); // Voltages a reverifier

  }
  public void lancer(){

    setVoltage(-4);

  }

  public void stop(){

    setVoltage(0);

  }


}
