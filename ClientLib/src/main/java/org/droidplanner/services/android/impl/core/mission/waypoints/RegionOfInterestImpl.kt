package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class RegionOfInterestImpl : SpatialCoordItem {
    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(mission: Mission?, coord: LatLongAlt?) : super(mission, coord) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission, null) {
        unpackMAVMessage(msg)
    }

    /**
     * @return True if this roi cancels a previously set roi.
     */
    val isReset: Boolean
        get() = coordinate.latitude == 0.0 && coordinate.longitude == 0.0 && coordinate.altitude == 0.0

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]

        // "Hurr durr, don't use DO_SET_ROI" Bullshit! These aren't implemented.
        // mavMsg.command = (isReset())? 
        // 	MAV_CMD.MAV_CMD_DO_SET_ROI_NONE: 
        // 	MAV_CMD.MAV_CMD_DO_SET_ROI_LOCATION;
        mavMsg.command = MAV_CMD.MAV_CMD_DO_SET_ROI
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        super.unpackMAVMessage(mavMsg)
    }

    override fun getType(): MissionItemType = MissionItemType.ROI
}
