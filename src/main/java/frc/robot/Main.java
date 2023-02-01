// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {} // overrides object constructor, only forces it private: keeps Main from being instantiated

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
