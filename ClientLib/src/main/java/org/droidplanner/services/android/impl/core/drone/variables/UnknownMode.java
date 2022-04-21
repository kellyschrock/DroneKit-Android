package org.droidplanner.services.android.impl.core.drone.variables;

public class UnknownMode implements BaseMode<UnknownMode> {
    public static final UnknownMode INSTANCE = new UnknownMode();

    @Override
    public String getName() {
        return "Unknown";
    }

    @Override
    public UnknownMode getNativeMode() {
        return INSTANCE;
    }
}
