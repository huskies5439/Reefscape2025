// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ascenseur extends SubsystemBase {
  
  public static final SparkFlexConfig moteurConfig = new SparkFlexConfig();
  private SparkFlex moteur = new SparkFlex(0, null);
  private ProfiledPIDController pidAscenseur = new ProfiledPIDController(0, 0, 0, 
    new TrapezoidProfile.Constraints(0,0));
  private double conversionEncoder;
  private double forceAscenseur;

  private final DigitalInput limitSwitch = new DigitalInput(0);

  private ArmFeedforward feedfoward = new ArmFeedforward(0,0,0);
  
  
  public Ascenseur() {

    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kBrake);
    moteur.configure(moteurConfig, null, null);
    
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Hauteur Ascenseur",0);
    forceAscenseur = SmartDashboard.getNumber("voltage ascenseur", 0);
  }

  public void monter(){
    moteur.setVoltage(1);
  }

  public void descendre(){
    moteur.setVoltage(-1);
  }

  public void stop(){
    moteur.setVoltage(0);
  }

  public ProfiledPIDController getAscenseuController(){
    return pidAscenseur;
  }

  
}
