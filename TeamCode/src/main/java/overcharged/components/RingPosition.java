package overcharged.components;

public enum RingPosition {
    A(2), //0 rings
    B(1), //1 ring
    C(0); //4 rings

    public int index;
    RingPosition(final int index) { this.index = index; }
}
