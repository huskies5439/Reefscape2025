// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class autoReculler extends ParallelCommandGroup {
  public autoReculler(    BasePilotable basePilotable,Ascenseur ascenseur,Poignet poignet) {

    addCommands(
      Commands.run(ascenseur::hold, ascenseur),
      Commands.run(poignet::hold,poignet),
      Commands.run(()->basePilotable.conduire(0.2, 0, 0,false, false), basePilotable)
    );
  }
}
