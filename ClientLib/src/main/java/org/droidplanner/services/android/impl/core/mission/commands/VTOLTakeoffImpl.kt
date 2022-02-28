package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.commands.MissionCMD
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class VTOLTakeoffImpl : MissionCMD {
    var frontTransitionHeading = 0
        private set
    var yawAngle = 0.0
        private set
    var lat = 0.0
        private set
    var lng = 0.0
        private set
    var alt = 0.0
        private set

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, frontTransitionHeading: Int, yawAngle: Double, lat: Double, lng: Double, alt: Double) : super(mission) {
        this.frontTransitionHeading = frontTransitionHeading
        this.yawAngle = yawAngle
        this.lat = lat
        this.lng = lng
        this.alt = alt
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF
        mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
        mavMsg.param1 = 0f
        mavMsg.param2 = frontTransitionHeading.toFloat()
        mavMsg.param3 = 0f
        mavMsg.param4 = yawAngle.toFloat()
        mavMsg.x = lat.toFloat()
        mavMsg.y = lng.toFloat()
        mavMsg.z = alt.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        frontTransitionHeading = mavMsg.param2.toInt()
        yawAngle = mavMsg.param4.toDouble()
        lat = mavMsg.x.toDouble()
        lng = mavMsg.y.toDouble()
        alt = mavMsg.z.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.VTOL_TAKEOFF

    companion object {
        const val DEFAULT_TAKEOFF_ALTITUDE = 10.0
    }
}
