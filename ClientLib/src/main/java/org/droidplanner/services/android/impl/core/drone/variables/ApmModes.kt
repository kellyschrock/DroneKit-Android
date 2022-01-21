package org.droidplanner.services.android.impl.core.drone.variables

import com.MAVLink.enums.MAV_TYPE
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import java.util.ArrayList

enum class ApmModes(val number: Long, val modeName: String, val type: Int) {
    FIXED_WING_MANUAL(0, "Manual", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_CIRCLE(1, "Circle", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_STABILIZE(2, "Stabilize", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_TRAINING(3, "Training", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_ACRO(4, "Acro", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_FLY_BY_WIRE_A(5, "FBW A", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_FLY_BY_WIRE_B(6, "FBW B", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_CRUISE(7, "Cruise", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_AUTOTUNE(8, "AutoTune", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_AUTO(10, "Auto", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_RTL(11, "RTL", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_LOITER(12, "Loiter", MAV_TYPE.MAV_TYPE_FIXED_WING),
    FIXED_WING_GUIDED(15, "Guided", MAV_TYPE.MAV_TYPE_FIXED_WING),
    ROTOR_STABILIZE(0, "Stabilize", MAV_TYPE.MAV_TYPE_QUADROTOR),

    ROTOR_ACRO(1, "Acro", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_ALT_HOLD(2, "Alt Hold", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_AUTO(3, "Auto", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_GUIDED(4, "Guided", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_LOITER(5, "Loiter", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_RTL(6, "RTL", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_CIRCLE(7, "Circle", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_LAND(9, "Land", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_TOY(11, "Drift", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_SPORT(13, "Sport", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_FLIP(14, "Flip", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_AUTOTUNE(15, "Autotune", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_POSHOLD(16, "PosHold", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_BRAKE(17, "Brake", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_THROW(18, "Throw", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_AVOID_ADSB(19, "Avoid ADSB", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_GUIDED_NOGPS(20, "Guided NoGPS", MAV_TYPE.MAV_TYPE_QUADROTOR),
    ROTOR_SMART_RTL(21, "Smart RTL", MAV_TYPE.MAV_TYPE_QUADROTOR),

    ROVER_MANUAL(0, "MANUAL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_ACRO(1, "ACRO", MAV_TYPE.MAV_TYPE_GROUND_ROVER),  //	ROVER_LEARNING(2, "LEARNING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_STEERING(3, "STEERING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_HOLD(4, "HOLD", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_LOITER(5, "LOITER", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_FOLLOW(6, "FOLLOW", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_SIMPLE(7, "SIMPLE", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_AUTO(10, "AUTO", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_RTL(11, "RTL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_SMARTRTL(12, "SMART_RTL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_GUIDED(15, "GUIDED", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
    ROVER_INITIALIZING(16, "INITIALIZING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),

    VTOL_STABILIZE(17, "QSTABILIZE", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
    VTOL_HOVER(18, "QHOVER", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
    VTOL_LOITER(19, "QLOITER", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
    VTOL_LAND(20, "QLAND", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),
    VTOL_RTL(21, "QRTL", MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR),

    UNKNOWN(-1, "Unknown", MAV_TYPE.MAV_TYPE_GENERIC);

    companion object {
        @JvmStatic
        fun getMode(i: Long, type: Int): ApmModes {
            var type = type
            if (isCopter(type)) {
                type = MAV_TYPE.MAV_TYPE_QUADROTOR
            } else if (isVtol(type)) {
                type = MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR
            }
            for (mode in values()) {
                if (i == mode.number && type == mode.type) {
                    return mode
                }
            }
            return UNKNOWN
        }

        @JvmStatic
        fun getMode(str: String, type: Int): ApmModes {
            var type = type
            if (isCopter(type)) {
                type = MAV_TYPE.MAV_TYPE_QUADROTOR
            } else if (isVtol(type)) {
                type = MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR
            }
            for (mode in values()) {
                if (str == mode.name && type == mode.type) {
                    return mode
                }
            }
            return UNKNOWN
        }

        @JvmStatic
        fun getModeList(type: Int): List<ApmModes> {
            var type = type
            if (isCopter(type)) {
                type = MAV_TYPE.MAV_TYPE_QUADROTOR
            } else if (isVtol(type)) {
                type = MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR
            }
            val modeList: MutableList<ApmModes> = ArrayList()
            for (mode in values()) {
                if (mode.type == type) {
                    modeList.add(mode)
                }
            }
            return modeList
        }

        @JvmStatic
		fun isValid(mode: ApmModes): Boolean {
            return mode != UNKNOWN
        }

        fun isCopter(type: Int): Boolean {
            return when (type) {
                MAV_TYPE.MAV_TYPE_TRICOPTER, MAV_TYPE.MAV_TYPE_QUADROTOR, MAV_TYPE.MAV_TYPE_HEXAROTOR, MAV_TYPE.MAV_TYPE_OCTOROTOR, MAV_TYPE.MAV_TYPE_HELICOPTER -> true
                else -> false
            }
        }

        @JvmStatic
        fun isVtol(type: Int): Boolean {
            return Type.isVtol(type)
        }
    }
}
