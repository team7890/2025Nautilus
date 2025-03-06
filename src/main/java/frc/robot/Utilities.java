package frc.robot;

public class Utilities {
    public static double limitVariable(double dMinValue, double dVariable, double dMaxValue) {
        double dValue;
        dValue = Math.max(dVariable, dMinValue);
        dValue = Math.min(dValue, dMaxValue);        
        return dValue;
    }
}
