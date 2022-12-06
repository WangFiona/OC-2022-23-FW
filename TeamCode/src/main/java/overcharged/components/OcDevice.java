package overcharged.components;

/**
 * Created on 11/22/2017.
 */
public class OcDevice
{
    protected String id;

    protected OcDevice(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }

    public String toString () {
        return id;
    }
}
