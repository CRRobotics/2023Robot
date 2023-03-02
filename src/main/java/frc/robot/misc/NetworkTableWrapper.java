package frc.robot.misc;

import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableWrapper {
    public static double getData(int table, String key) {
        return NetworkTableInstance.getDefault().getTable(String.valueOf(table)).getEntry(key).getDouble(0);
    }
}
