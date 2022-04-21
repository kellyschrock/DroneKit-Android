package org.droidplanner.services.android.impl.core.drone.variables;

public interface BaseMode<NativeType> {
    String getName();

    NativeType getNativeMode();
}
