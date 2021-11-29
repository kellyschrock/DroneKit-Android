package org.droidplanner.services.android.impl.core.MAVLink.connection;

import java.util.HashSet;

import com.o3dr.services.android.lib.model.ICommandListener;

class MavLinkConnectionSubscribers {

    private final HashSet<MavLinkConnectionListener> listeners = new HashSet<>();

    private static MavLinkConnectionSubscribers instance;
    private MavLinkConnectionSubscribers() { }

    public static MavLinkConnectionSubscribers get() {
        if(instance == null)
            instance = new MavLinkConnectionSubscribers();
        return instance;
    }

    public void registerListener(MavLinkConnectionListener listener) {
        listeners.add(listener);
    }

    public void unregisterListener(MavLinkConnectionListener listener) {
        listeners.remove(listener);
    }

    public HashSet<MavLinkConnectionListener> getListeners() {
        return listeners;
    }
}
