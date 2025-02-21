// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.FollowPathVariable;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Poignet;


public class AutoStationVariable extends ParallelCommandGroup {
  
  public AutoStationVariable(boolean isProcesseur, DoubleSupplier joystickSupplier, BasePilotable basePilotable,Ascenseur ascenseur, Poignet poignet, CorailManip corailManip ) {
   //se rend automatiquement a la station de Corail

    addCommands(
      new FollowPathVariable(isProcesseur, joystickSupplier, basePilotable).andThen(
      Commands.run(basePilotable::setX, basePilotable)
      ),
      
      
      new SequentialCommandGroup(
        new WaitUntilCommand(()->{
          return basePilotable.isProcheStationCage() || basePilotable.isProcheStationProcesseur();
          }
        ),
         new GoToHauteur(()-> Hauteur.station[0], ()-> Hauteur.station[1], ascenseur, poignet).alongWith(corailManip.goberCommand())
      )
    );
  }
}
