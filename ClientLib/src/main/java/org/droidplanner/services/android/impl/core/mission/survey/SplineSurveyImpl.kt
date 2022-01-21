package org.droidplanner.services.android.impl.core.mission.survey

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class SplineSurveyImpl(mission: Mission?, points: List<LatLong?>?) : SurveyImpl(mission, points) {
    override fun getSurveyPoint(point: LatLong, altitude: Double): msg_mission_item {
        return msg_mission_item().apply {
            autocontinue = 1
            frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
            command = MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT
            x = point.latitude.toFloat()
            y = point.longitude.toFloat()
            z = altitude.toFloat()
            param1 = 0f
            param2 = 0f
            param3 = 0f
            param4 = 0f
        }
    }

    override fun getType(): MissionItemType = MissionItemType.SPLINE_SURVEY
}
