package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class PositionsLift {
    public static int ground = 0;
    public static int park = 340;
    public static int transfer = 0;
    public static int highChamber = 508;
    public static int scoreSpecimen = 800;
    public static int highBasket = 1210;
    public static int highBasket2 = 1235;
    public static int lowBasket = 600;

    public static double kP = 0.01;
    public static double kP2 = 0.021;
    public static double kI = 0;
    public static double kD = 0;
}    
