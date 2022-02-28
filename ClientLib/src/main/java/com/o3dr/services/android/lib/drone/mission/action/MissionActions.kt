package com.o3dr.services.android.lib.drone.mission.action

import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
object MissionActions {
    const val ACTION_GENERATE_DRONIE = Utils.PACKAGE_NAME + ".action.GENERATE_DRONIE"
    const val ACTION_SET_MISSION = Utils.PACKAGE_NAME + ".action.SET_MISSION"
    const val ACTION_START_MISSION = Utils.PACKAGE_NAME + ".action.START_MISSION"
    const val ACTION_GOTO_WAYPOINT = Utils.PACKAGE_NAME + ".action.GOTO_WAYPOINT"
    const val EXTRA_MISSION = "extra_mission"
    const val EXTRA_MISSION_ITEM_INDEX = "extra_mission_item_index"
    const val EXTRA_REPEAT_COUNT = "extra_repeat_count"
    const val EXTRA_PUSH_TO_DRONE = "extra_push_to_drone"
    const val EXTRA_FORCE_MODE_CHANGE = "extra_force_mode_change"
    const val EXTRA_FORCE_ARM = "extra_force_arm"
    const val EXTRA_MISSION_SPEED = "extra_mission_speed"
    const val ACTION_LOAD_WAYPOINTS = Utils.PACKAGE_NAME + ".action.LOAD_WAYPOINTS"
    const val ACTION_BUILD_COMPLEX_MISSION_ITEM = Utils.PACKAGE_NAME + ".action" +
            ".BUILD_COMPLEX_MISSION_ITEM"
    const val ACTION_CHANGE_MISSION_SPEED = Utils.PACKAGE_NAME + ".action" +
            ".CHANGE_MISSION_SPEED"
}
