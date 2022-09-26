// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(9.39)*2; 
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(9.39)*2; 

    public static final int DRIVETRAIN_PIGEON_ID = 20;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; 
    public static final int FRONT_LEFT_MODULE_SPEED_ENCODER = 14;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(187.8-90); //-2

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; 
    public static final int FRONT_RIGHT_MODULE_SPEED_ENCODER = 13; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(148.0-90); //-2

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
    public static final int BACK_LEFT_MODULE_SPEED_ENCODER = 16;   
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(205.5-90); //+4

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR =6; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; 
    public static final int BACK_RIGHT_MODULE_SPEED_ENCODER = 15; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(241.7-90); //+2

    

  
}
