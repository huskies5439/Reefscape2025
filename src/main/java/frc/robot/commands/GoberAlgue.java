// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgueManip;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoberAlgue extends Command {
  private AlgueManip algueManip;
  public GoberAlgue(AlgueManip algueManip) {
    this.algueManip = algueManip;
    addRequirements(algueManip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algueManip.gober();
  }


  @Override
  public void end(boolean interrupted) {
    algueManip.stop();
  }

  @Override
  public boolean isFinished() {
    return algueManip.isAlgue();
  }
}
