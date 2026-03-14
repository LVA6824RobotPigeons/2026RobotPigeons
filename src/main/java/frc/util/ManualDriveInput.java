package frc.util;

public class ManualDriveInput {
    public final double forward;
    public final double left;
    public final double rotation;

    public ManualDriveInput(double forward, double left, double rotation) {
        this.forward = forward;
        this.left = left;
        this.rotation = rotation;
    }

    public ManualDriveInput() {
        this(0, 0, 0);
    }

    public boolean hasTranslation() {
        return (left != 0.0) || (forward != 0.0);
    }

    public boolean hasRotation() {
        return Math.abs(rotation) > 0;
    }
}
