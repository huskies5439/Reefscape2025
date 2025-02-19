// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.GamePositions;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;


public class AutoStation extends ParallelCommandGroup {
  
  public AutoStation(Pose2d cible, BasePilotable basePilotable,Ascenseur ascenseur, Poignet poignet  ) {
   //se rend automatiquement a la station de Corail

    addCommands(
      basePilotable.followPath(cible),
      
      new SequentialCommandGroup(
        // new WaitUntilCommand(()->{
        //   return basePilotable.isProcheStationCage() || basePilotable.isProcheStationProcesseur();
        //   }
        // ),
        // new GoToHauteur(()-> Hauteur.station[0], ()-> Hauteur.station[1], ascenseur, poignet)
      )
    );
  }
}
