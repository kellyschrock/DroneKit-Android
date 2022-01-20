package org.droidplanner.services.android.impl.core.drone.autopilot.px4

import android.content.Context
import android.os.Handler
import com.MAVLink.Messages.MAVLinkMessage
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkProvider
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser
import org.droidplanner.services.android.impl.core.drone.LogMessageListener
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone
import org.droidplanner.services.android.impl.core.firmware.FirmwareType

/**
 * Created by Fredia Huya-Kouadio on 9/10/15.
 */
class Px4Native(droneId: String?, context: Context?, handler: Handler?, mavClient: DataLinkProvider<MAVLinkMessage?>?, warningParser: AutopilotWarningParser?, logListener: LogMessageListener?) : GenericMavLinkDrone(droneId, context, handler, mavClient, warningParser, logListener) {
    override val firmwareType: FirmwareType
        get() = FirmwareType.PX4_NATIVE
}
