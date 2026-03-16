// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.1;
  }
  public static final double maxSpeed = Units.inchesToMeters(2);

  public static class FuelConstants
  {
    public static final int ShooterMotorLeftID = 1;
    public static final int ShooterMotorMidID = 2;
    public static final int ShooterMotorRightID = 3;
    public static final int maxVoltage = 35;
    public static final int IndexerMotorID = 4;
    public static final int FeederMotorID = 5;
    public static final int SpinUpSec = 2;

    public static final int kHoodLeftServo = 1;
    public static final int kHoodRightServo = 2;

    public static final double kVelocityTolerance = 100;

    public static class PIDConstants 
    {
      public static final double ShooterKp = 0;
      public static final double ShooterKi = 0;
      public static final double ShooterKd = 0;
      
      public static final double ShooterKMinOutput = 0;
      public static final double ShooterKMaxOutput = 0;
    }
  }
}
