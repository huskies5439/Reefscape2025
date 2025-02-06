// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CorailManip;

//!!N'UTILISE PAS LE CAPTEUR!!
public class PlacerCorail extends Command {
  private CorailManip corailManip;
  public PlacerCorail(CorailManip corailManip) {
    this.corailManip = corailManip;
    addRequirements(corailManip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    corailManip.sortir();
  }

  @Override
  public void end(boolean interrupted) {
    corailManip.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
