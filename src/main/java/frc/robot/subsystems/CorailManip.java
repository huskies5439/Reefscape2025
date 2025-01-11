// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CorailManip extends SubsystemBase {
  /** Creates a new CorailManip. */

  private final SparkMax moteur = new SparkMax(12, MotorType.kBrushless); // ID a reverifier 
  public static final SparkMaxConfig configMoteur = new SparkMaxConfig(); 

  public final DigitalInput capteur = new DigitalInput(3); // Channel a reverifier


  public CorailManip() {
    configMoteur.inverted(false);
    configMoteur.idleMode(IdleMode.kCoast); 
    moteur.configure(configMoteur, null, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isCorail(){
    return !capteur.get(); 
  }

  public void setVoltage(double voltage){
    moteur.setVoltage(voltage);
  }

  public void stop(){
    setVoltage(0);
  }

  public void gober(){
   setVoltage(4);
    
  }

  public void lancer(){
    
    setVoltage(-4); 
    
  }


}
