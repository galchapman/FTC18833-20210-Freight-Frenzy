package edu.megiddo.lions;

public enum DistanceUnit {
    Inch(0.0254),
    Feet(0.0254 * 12),
    Meter(1),
    CentiMeters(0.01),
    MilliMeters(0.001);

    double metersRatio;
    DistanceUnit(double ratio) {
        metersRatio = ratio;
    }

    public double convertTo(DistanceUnit unit, double value) {
        return value * metersRatio / unit.metersRatio;
    }
}
