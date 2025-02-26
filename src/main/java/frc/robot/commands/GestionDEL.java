// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.Key;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.DEL;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GestionDEL extends Command {

  DEL del; 
  CorailManip corailManip; 
  AlgueManip algueManip; 
  BasePilotable basePilotable; 
  
  public GestionDEL(DEL del, CorailManip corailManip, AlgueManip algueManip,BasePilotable basePilotable) {
    this.del = del; 
    this.corailManip = corailManip; 
    this.algueManip = algueManip; 
    this.basePilotable = basePilotable; 
    addRequirements(del); 
  }

  
  @Override
  public void initialize() {
   
  }


  @Override
  public void execute() {
    if(corailManip.isCorail() || algueManip.isAlgue()){
      del.couleur(Color.kGreen);
    }else{
      if(basePilotable.isRedAlliance()){
        del.breathe(Color.kRed);
      }else{
        del.breathe(Color.kBlue);
      }
    }
   
  }


  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return false;
  }
}
