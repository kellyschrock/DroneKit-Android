package org.droidplanner.services.android.impl.core.drone.variables;

import com.MAVLink.enums.MAV_MODE_FLAG;
import com.MAVLink.enums.MAV_TYPE;
import com.o3dr.services.android.lib.drone.property.VehicleMode;

import java.nio.ByteBuffer;
import java.util.HashMap;

public class Px4Util {
    public static final int AUTO_MODE_FLAGS =
            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;

    public static final int CUSTOM_ENABLED = MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    public static final int RAW_MODE_FLAGS =
            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    public static final int MANUAL_MODE_FLAGS =
            RAW_MODE_FLAGS | MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;

    public static final int PX4_CUSTOM_MAIN_MODE_MANUAL            = 1;
    public static final int PX4_CUSTOM_MAIN_MODE_ALTCTL            = 2;
    public static final int PX4_CUSTOM_MAIN_MODE_POSCTL            = 3;
    public static final int PX4_CUSTOM_MAIN_MODE_AUTO              = 4;
    public static final int PX4_CUSTOM_MAIN_MODE_ACRO              = 5;
    public static final int PX4_CUSTOM_MAIN_MODE_OFFBOARD          = 6;
    public static final int PX4_CUSTOM_MAIN_MODE_STABILIZED        = 7;
    public static final int PX4_CUSTOM_MAIN_MODE_RATTITUDE         = 8;

    public static final int PX4_CUSTOM_SUB_MODE_AUTO_READY         = 1;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF       = 2;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_LOITER        = 3;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_MISSION       = 4;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_RTL           = 5;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_LAND          = 6;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_RTGS          = 7;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8;

    private static final HashMap<VehicleMode, Px4Mode> sModeMap = new HashMap<>();

    public static class Px4CustomMode {
        public int reserved;
        public short main_mode;
        public short sub_mode;

        public Px4CustomMode(long input) {
            final ByteBuffer buf = ByteBuffer.allocate(8);
            buf.putLong(input);
            buf.rewind();
            reserved = buf.getInt();
            main_mode = buf.get();
            sub_mode = buf.get();
        }

        @Override
        public String toString() {
            return "Px4CustomMode{" +
                    "reserved=" + reserved +
                    ", main_mode=" + main_mode +
                    ", sub_mode=" + sub_mode +
                    '}';
        }
    }

    static {
        // Copter
        sModeMap.put(VehicleMode.COPTER_PX4_MANUAL, Px4Mode.MANUAL);
        sModeMap.put(VehicleMode.COPTER_PX4_RATTITUDE, Px4Mode.RATTITUDE);
        sModeMap.put(VehicleMode.COPTER_STABILIZE, Px4Mode.STABILIZED);
        sModeMap.put(VehicleMode.COPTER_ACRO, Px4Mode.ACRO);
        sModeMap.put(VehicleMode.COPTER_ALT_HOLD, Px4Mode.ALTCTL);
        sModeMap.put(VehicleMode.COPTER_AUTO, Px4Mode.MISSION);
        sModeMap.put(VehicleMode.COPTER_LOITER, Px4Mode.POSCTL);
        sModeMap.put(VehicleMode.COPTER_RTL, Px4Mode.RTL);
        sModeMap.put(VehicleMode.COPTER_SMART_RTL, Px4Mode.RTL);
        sModeMap.put(VehicleMode.COPTER_CIRCLE, Px4Mode.POSCTL);
        sModeMap.put(VehicleMode.COPTER_GUIDED, Px4Mode.OFFBOARD);

        // Plane
        sModeMap.put(VehicleMode.PLANE_MANUAL, Px4Mode.MANUAL);
        sModeMap.put(VehicleMode.PLANE_ACRO, Px4Mode.ACRO);
        sModeMap.put(VehicleMode.PLANE_STABILIZE, Px4Mode.STABILIZED);
        sModeMap.put(VehicleMode.PLANE_TRAINING, Px4Mode.STABILIZED);
        sModeMap.put(VehicleMode.PLANE_FLY_BY_WIRE_A, Px4Mode.ALTCTL);
        sModeMap.put(VehicleMode.PLANE_FLY_BY_WIRE_B, Px4Mode.POSCTL);
        sModeMap.put(VehicleMode.PLANE_CRUISE, Px4Mode.POSCTL);
        sModeMap.put(VehicleMode.PLANE_AUTO, Px4Mode.MISSION);
        sModeMap.put(VehicleMode.PLANE_RTL, Px4Mode.RTL);
        sModeMap.put(VehicleMode.PLANE_LOITER, Px4Mode.LOITER);
        sModeMap.put(VehicleMode.PLANE_GUIDED, Px4Mode.OFFBOARD);

        // Rover
        sModeMap.put(VehicleMode.ROVER_MANUAL, Px4Mode.MANUAL);
        sModeMap.put(VehicleMode.ROVER_ACRO, Px4Mode.ACRO);
        sModeMap.put(VehicleMode.ROVER_LEARNING, Px4Mode.STABILIZED);
        sModeMap.put(VehicleMode.ROVER_STEERING, Px4Mode.STABILIZED);
        sModeMap.put(VehicleMode.ROVER_AUTO, Px4Mode.MISSION);
        sModeMap.put(VehicleMode.ROVER_RTL, Px4Mode.RTL);
        sModeMap.put(VehicleMode.ROVER_SMART_RTL, Px4Mode.RTL);
        sModeMap.put(VehicleMode.ROVER_GUIDED, Px4Mode.OFFBOARD);
    }

    static VehicleMode toPlaneMode(Px4Mode mode) {
        switch(mode) {
            case MANUAL: return VehicleMode.PLANE_MANUAL;
            case ACRO: return VehicleMode.PLANE_ACRO;
            case STABILIZED: return VehicleMode.PLANE_STABILIZE;
            case ALTCTL: return VehicleMode.PLANE_FLY_BY_WIRE_A;
            case POSCTL: return VehicleMode.PLANE_FLY_BY_WIRE_B;
            case MISSION: return VehicleMode.PLANE_AUTO;
            case RTL: return VehicleMode.PLANE_RTL;
            case LOITER: return VehicleMode.PLANE_LOITER;
            case OFFBOARD: return VehicleMode.PLANE_GUIDED;
            case FOLLOW_ME:
            default: {
                return VehicleMode.UNKNOWN;
            }
        }
    }

    static VehicleMode toCopterMode(Px4Mode mode) {
        switch(mode) {
            case MANUAL: return VehicleMode.COPTER_PX4_MANUAL;
            case STABILIZED: return VehicleMode.COPTER_STABILIZE;
            case RATTITUDE: return VehicleMode.COPTER_PX4_RATTITUDE;
            case ACRO: return VehicleMode.COPTER_ACRO;
            case ALTCTL: return VehicleMode.COPTER_ALT_HOLD;
            case MISSION: return VehicleMode.COPTER_AUTO;
            case LOITER: return VehicleMode.COPTER_LOITER;
            case POSCTL: return VehicleMode.COPTER_POSHOLD;
            case RTL: return VehicleMode.COPTER_RTL;
            case OFFBOARD: return VehicleMode.COPTER_GUIDED;
            case FOLLOW_ME: // Not supported anyway, leave it
            default: {
                return VehicleMode.UNKNOWN;
            }
        }
    }

    static Px4Mode toPx4Mode(VehicleMode mode, int mavType) {
        Px4Mode px4Mode = sModeMap.get(mode);
        if(px4Mode == null) {
            // Unsupported mode
            switch(mavType) {
                case MAV_TYPE.MAV_TYPE_QUADROTOR: {
                    px4Mode = Px4Mode.POSCTL;
                    break;
                }

                case MAV_TYPE.MAV_TYPE_FIXED_WING:
                case MAV_TYPE.MAV_TYPE_GROUND_ROVER:
                case MAV_TYPE.MAV_TYPE_SURFACE_BOAT:
                default: {
                    px4Mode = Px4Mode.STABILIZED;
                    break;
                }
            }
        }

        return px4Mode;
    }
}
