// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.CorailManip;

//!!N'UTILISE PAS LE CAPTEUR!!
public class PlacerAlgue extends Command {
  private AlgueManip algueManip;
  public PlacerAlgue(AlgueManip algueManip) {
    this.algueManip = algueManip;
    addRequirements(algueManip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algueManip.sortir();
  }

  @Override
  public void end(boolean interrupted) {
    algueManip.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
