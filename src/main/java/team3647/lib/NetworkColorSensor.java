// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.lib;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public final class NetworkColorSensor {
    public final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    public final NetworkTableEntry proximityEntry;
    public final NetworkTableEntry rawColorsEntry;

    private final double[] kEmptyDoubleThree = new double[3];
    private final int kMaxReadDistance;

    private static final int kRedIndex = 0;
    private static final int kGreenIndex = 1;
    private static final int kBlueIndex = 2;

    public enum GamePiece {
        NONE("None"),
        CONE("Cone"),
        CUBE("Cube");

        public final String str;

        private GamePiece(String name) {
            str = name;
        }
    }

    private double proximity;
    private double[] rawColors = new double[3];

    public NetworkColorSensor(String proximity, String color, int maxReadDistance) {
        this.proximityEntry = networkTableInstance.getEntry(proximity);
        this.rawColorsEntry = networkTableInstance.getEntry(color);

        kMaxReadDistance = maxReadDistance;
    }

    public synchronized double getProximity() {
        return proximityEntry.getDouble(0.0);
    }

    public synchronized boolean isReadColor() {
        return this.proximity > kMaxReadDistance;
    }

    private static GamePiece updateColor(
            double red, double green, double blue, boolean withinDistance) {
        GamePiece result;
        if (green > red && green > blue || !withinDistance) {
            result = GamePiece.NONE;
        } else if (blue > red) {
            result = GamePiece.CONE;
        } else {
            result = GamePiece.CUBE;
        }
        return result;
    }

    public GamePiece getGamepiece() {
        var rawColors = rawColorsEntry.getDoubleArray(kEmptyDoubleThree);
        return updateColor(
                rawColors[kRedIndex], rawColors[kGreenIndex], rawColors[kBlueIndex], isReadColor());
    }
}
