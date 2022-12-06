package overcharged.components;

public enum RotationAxis {
    RIGHT(2),
    CENTER(1),
    LEFT(0);

    public int index;
    RotationAxis(final int index) { this.index = index; }
}
