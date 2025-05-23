package frc.robot.Util;

import edu.wpi.first.math.geometry.Twist2d;

public class EpsilonComparison {
    private static double kEpsilson = 1E-12;

    public static boolean compare(double a, double b, double epsilson){
        return(a - epsilson <= b) && (a + epsilson >= b);
    }

    public static boolean compare(double a, double b){
        return compare(a, b, kEpsilson);
    }

    public static boolean compare(Twist2d a, Twist2d b){
        return compare(a.dx,b.dx) && compare(a.dy,b.dy) && compare(a.dtheta,b.dtheta);
    } 

    public static boolean compare(Twist2d a, Twist2d b, double epsilson){
        return compare(a.dx,b.dx,epsilson) && compare(a.dy,b.dy,epsilson) && compare(a.dtheta,b.dtheta,epsilson);
    } 

    

}
