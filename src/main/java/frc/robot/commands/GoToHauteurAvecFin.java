package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;


public class GoToHauteurAvecFin extends ParallelRaceGroup {

  //Une version de GoToHauteur qui se termine automatiquement quand on arrive aux deux cibles
  //Existe seulement pour éviter d'avoir à répéter un .until() sur GoToHauteur plusieurs fois
  public GoToHauteurAvecFin(DoubleSupplier cibleAscenceur, DoubleSupplier ciblePoignet, Ascenseur ascenseur, Poignet poignet) {

    addCommands(
      new GoToHauteur(cibleAscenceur, ciblePoignet, ascenseur, poignet),//Ne finit jamais
      new WaitUntilCommand(()-> {return ascenseur.atCible() && poignet.atCible();})//On arrête quand on atteint les deux cibles
    );
  }
}
