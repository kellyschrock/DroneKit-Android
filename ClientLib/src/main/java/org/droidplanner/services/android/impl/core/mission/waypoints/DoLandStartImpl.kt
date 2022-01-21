package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.waypoints.SpatialCoordItem
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class DoLandStartImpl : SpatialCoordItem {
    constructor(item: MissionItemImpl?) : super(item) {
        setAltitude(0.0)
    }

    constructor(mission: Mission?) : this(mission, LatLong(0.0, 0.0)) {}
    constructor(mMission: Mission?, coord: LatLong?) : super(mMission, LatLongAlt(coord, 0.0)) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission, null) {
        unpackMAVMessage(msg)
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_LAND_START
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        super.unpackMAVMessage(mavMsg)
    }

    override fun getType(): MissionItemType = MissionItemType.DO_LAND_START
}
