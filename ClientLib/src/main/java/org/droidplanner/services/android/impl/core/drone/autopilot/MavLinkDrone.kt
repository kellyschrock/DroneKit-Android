package org.droidplanner.services.android.impl.core.drone.autopilot

import com.MAVLink.Messages.MAVLinkMessage
import org.droidplanner.services.android.impl.core.drone.profiles.ParameterManager
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkProvider
import org.droidplanner.services.android.impl.core.MAVLink.WaypointManager
import org.droidplanner.services.android.impl.core.drone.variables.calibration.AccelCalibration
import org.droidplanner.services.android.impl.core.drone.variables.calibration.MagnetometerCalibrationImpl
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.*
import org.droidplanner.services.android.impl.core.mission.Mission

interface MavLinkDrone : Drone {
    val isConnectionAlive: Boolean
    val mavlinkVersion: Int
    fun onMavLinkMessageReceived(message: MAVLinkMessage?)
    val sysid: Short
    val compid: Short
    val state: State?
    val parameterManager: ParameterManager?
    val type: Int
    val firmwareType: FirmwareType?
    val mavClient: DataLinkProvider<MAVLinkMessage?>?
    val waypointManager: WaypointManager?
    val mission: Mission?
    val streamRates: StreamRates?
    val missionStats: MissionStats?
    val guidedPoint: GuidedPoint?
    val calibrationSetup: AccelCalibration?
    val magnetometerCalibration: MagnetometerCalibrationImpl?
    val firmwareVersion: String?
    val camera: Camera?

    companion object {
        const val PACKAGE_NAME = "org.droidplanner.services.android.core.drone.autopilot"
        const val ACTION_REQUEST_HOME_UPDATE = "$PACKAGE_NAME.action.REQUEST_HOME_UPDATE"
    }
}
