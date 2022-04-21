package org.droidplanner.services.android.impl.core.drone.autopilot.px4;

import android.content.Context;
import android.os.Handler;

import com.MAVLink.Messages.MAVLinkMessage;

import org.droidplanner.services.android.impl.communication.model.DataLink;
import org.droidplanner.services.android.impl.core.drone.LogMessageListener;
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser;

public class Px4Copter extends Px4Native {
    public Px4Copter(String droneId, Context context, Handler handler, DataLink.DataLinkProvider<MAVLinkMessage> mavClient, AutopilotWarningParser warningParser, LogMessageListener logListener) {
        super(droneId, context, handler, mavClient, warningParser, logListener);
    }
}
