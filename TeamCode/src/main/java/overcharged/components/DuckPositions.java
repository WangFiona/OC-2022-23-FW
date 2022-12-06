package overcharged.components;

public enum DuckPositions {
    A(2), //Left, First level
    B(1), //Middle, Second level
    C(0); //Right, Third level

    public int index;
    DuckPositions(final int index) { this.index = index; }
}
