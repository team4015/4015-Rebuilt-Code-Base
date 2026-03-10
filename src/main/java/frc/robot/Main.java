// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot entry point used by WPILib to start the main {@link Robot} class.
 *
 * <p>Keep this class minimal and avoid placing robot initialization here.</p>
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function.
   *
   * <p>Starts the WPILib robot lifecycle using {@link Robot}.</p>
   *
   * @param args command-line arguments (unused)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
