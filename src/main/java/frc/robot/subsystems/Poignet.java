// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

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
  /** Creates a new Poignet. */
  public final SparkFlex moteur = new SparkFlex(13, MotorType.kBrushless);
  public static final SparkFlexConfig configMoteur = new SparkFlexConfig();
  public final DigitalInput capteur = new DigitalInput(4);// Channel a reverifier
  private ProfiledPIDController pidPoignet = new ProfiledPIDController( 0.1, 0, 0, 
  new TrapezoidProfile.Constraints(320,420));


  private ArmFeedforward feedforward = new ArmFeedforward(0,0,0);

  public Poignet() {
    configMoteur.inverted(false);
    configMoteur.idleMode(IdleMode.kBrake);
    moteur.configure(configMoteur, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle Poignet", getPosition());
  }

  public void setVoltage(double voltage){
    moteur.setVoltage(voltage);
  }

  public void monter(){
    setVoltage(1);
  }

  public void descendre(){
    setVoltage(-1);
  }

  public void stop(){
    setVoltage(0);
  }

  public double getPosition(){
    return moteur.getEncoder().getPosition();
  }

  public void setPID(double cible){
    double voltagePID = pidPoignet.calculate(getPosition(),cible);

    double voltageFF =  feedforward.calculate(
      Math.toRadians(getPosition()),
      pidPoignet.getSetpoint().velocity
      );

    setVoltage(voltagePID + voltageFF);

  }

  public boolean atCible(){
    return pidPoignet.atGoal();
  }

  public void resetEncoders(){
    moteur.getEncoder().setPosition(0);
  }

}
