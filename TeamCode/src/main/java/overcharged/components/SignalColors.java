package overcharged.components;

public enum SignalColors {
    Red(2), //Red
    Green(1), //Green
    Blue(0); //Blue

    public int index;
    SignalColors(final int index) { this.index = index; }
}
