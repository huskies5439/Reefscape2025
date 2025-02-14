// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class GoToHauteur extends Command {
  private DoubleSupplier cibleAscenceur;
  private DoubleSupplier ciblePoignet;
  private double ciblePoignetProtege;
  private double cibleAngleMin;
  private Ascenseur ascenseur;
  private Poignet poignet;


  //Double PID pour le poignet et l'ascenseur
  //que faire après la command? tel est la question
  public GoToHauteur(DoubleSupplier cibleAscenceur, DoubleSupplier ciblePoignet, Ascenseur ascenseur, Poignet poignet) {
    this.cibleAscenceur = cibleAscenceur;
    this.ciblePoignet = ciblePoignet;
    this.ascenseur = ascenseur;
    this.poignet = poignet;
    addRequirements(ascenseur, poignet);
  }

  @Override
  public void initialize() {
    ascenseur.resetPID();
    poignet.resetPID();

  }

  @Override
  public void execute() {

    ciblePoignetProtege=ciblePoignet.getAsDouble();

    //cibleAngleMin=-Math.toDegrees(Math.asin(ascenseur.getPositionVortex()+0.15)); calcul à revoir
    //ciblePoignetProtege = MathUtil.clamp(ciblePoignetProtege,-90 , 90);

    ascenseur.setPID(cibleAscenceur.getAsDouble());
    poignet.setPID(ciblePoignetProtege);

  }

  @Override
  public void end(boolean interrupted) {
    ascenseur.hold();
    poignet.hold();
  }

  @Override
  public boolean isFinished() {
    return ascenseur.atCible() && poignet.atCible();
  }
}
