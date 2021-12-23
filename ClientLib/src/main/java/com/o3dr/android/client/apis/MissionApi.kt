package com.o3dr.android.client.apis

import com.o3dr.services.android.lib.drone.mission.action.MissionActions
import android.os.Bundle
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.MissionItem.ComplexItem
import com.o3dr.android.client.apis.MissionApi
import com.o3dr.services.android.lib.drone.mission.Mission
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides access to missions specific functionality.
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
class MissionApi private constructor(private val drone: Drone) : Api() {
    /**
     * Generate action to create a dronie mission, and upload it to the connected drone.
     */
    fun generateDronie() {
        drone.performAsyncAction(Action(MissionActions.ACTION_GENERATE_DRONIE))
    }

    /**
     * Generate action to update the mission property for the drone model in memory.
     *
     * @param mission     mission to upload to the drone.
     * @param pushToDrone if true, upload the mission to the connected device.
     */
    fun setMission(mission: Mission?, pushToDrone: Boolean) {
        val params = Bundle()
        params.putParcelable(MissionActions.EXTRA_MISSION, mission)
        params.putBoolean(MissionActions.EXTRA_PUSH_TO_DRONE, pushToDrone)
        drone.performAsyncAction(Action(MissionActions.ACTION_SET_MISSION, params))
    }

    /**
     * Starts the mission. The vehicle will only accept this command if armed and in Auto mode.
     * note: This command is only supported by APM:Copter V3.3 and newer.
     *
     * @param forceModeChange Change to Auto mode if not in Auto.
     * @param forceArm Arm the vehicle if it is disarmed.
     * @param listener
     */
    fun startMission(forceModeChange: Boolean, forceArm: Boolean, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putBoolean(MissionActions.EXTRA_FORCE_MODE_CHANGE, forceModeChange)
        params.putBoolean(MissionActions.EXTRA_FORCE_ARM, forceArm)
        drone.performAsyncActionOnDroneThread(Action(MissionActions.ACTION_START_MISSION, params), listener)
    }

    /**
     * Jump to the desired command in the mission list. Repeat this action only the specified number of times
     * @param waypoint command to jump to
     * @param listener
     */
    fun gotoWaypoint(waypoint: Int, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putInt(MissionActions.EXTRA_MISSION_ITEM_INDEX, waypoint)
        drone.performAsyncActionOnDroneThread(Action(MissionActions.ACTION_GOTO_WAYPOINT, params), listener)
    }

    /**
     * Load waypoints from the target vehicle.
     */
    fun loadWaypoints() {
        drone.performAsyncAction(Action(MissionActions.ACTION_LOAD_WAYPOINTS))
    }

    /**
     * Build and return complex mission item.
     * @param itemBundle bundle containing the complex mission item to update.
     */
    private fun buildComplexMissionItem(itemBundle: Bundle): Action? {
        val payload = Action(MissionActions.ACTION_BUILD_COMPLEX_MISSION_ITEM, itemBundle)
        val result = drone.performAction(payload)
        return if (result) payload else null
    }

    /**
     * Builds and validates a complex mission item against the target vehicle.
     * @param complexItem Mission item to build.
     * @return an updated mission item.
     */
    fun <T : MissionItem?> buildMissionItem(complexItem: ComplexItem<T>): T? {
        val missionItem = complexItem as T
        val payload = missionItem!!.type?.storeMissionItem(missionItem) ?: return null
        val result = buildComplexMissionItem(payload)
        return if (result != null) {
            val updatedItem: T = MissionItemType.restoreMissionItemFromBundle(result.data)
            complexItem.copy(updatedItem)
            complexItem
        } else null
    }

    /**
     * Stops the vehicle at the current location. The vehicle will remain in Auto mode
     * @param listener
     *
     * @since 2.8.0
     */
    fun pauseMission(listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putFloat(MissionActions.EXTRA_MISSION_SPEED, 0f)
        drone.performAsyncActionOnDroneThread(Action(MissionActions.ACTION_CHANGE_MISSION_SPEED, params), listener)
    }

    /**
     * Sets the mission to a specified speed
     * @param speed Speed to set mission in m/s
     * @param listener
     *
     * @since 2.8.0
     */
    fun setMissionSpeed(speed: Float, listener: AbstractCommandListener?) {
        val params = Bundle()
        params.putFloat(MissionActions.EXTRA_MISSION_SPEED, speed)
        drone.performAsyncActionOnDroneThread(Action(MissionActions.ACTION_CHANGE_MISSION_SPEED, params), listener)
    }

    companion object {
        private val missionApiCache = ConcurrentHashMap<Drone, MissionApi>()
        private val apiBuilder: Builder<MissionApi> = Builder { drone -> MissionApi(drone) }

        /**
         * Retrieves a MissionApi instance.
         * @param drone Target vehicle
         * @return a MissionApi instance.
         */
        @JvmStatic
        fun getApi(drone: Drone?): MissionApi {
            return getApi(drone, missionApiCache, apiBuilder)
        }
    }
}
