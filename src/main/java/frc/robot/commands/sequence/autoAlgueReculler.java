// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoReculler;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class autoAlgueReculler extends SequentialCommandGroup {
  public autoAlgueReculler(Pose2d cible, double[] hauteur, Ascenseur ascenseur, Poignet poignet, BasePilotable basePilotable, AlgueManip algueManip) {

    addCommands(
      new AutoAlgue(cible, hauteur, ascenseur, poignet, basePilotable, algueManip).until(algueManip::isAlgue),
      new autoReculler(basePilotable,ascenseur,poignet).withTimeout(0.5)
    );
  }
}
