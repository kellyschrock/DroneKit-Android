package org.droidplanner.services.android.impl.core.drone.variables;

import com.MAVLink.minimal.msg_heartbeat;
import com.MAVLink.enums.MAV_TYPE;
import com.o3dr.services.android.lib.drone.property.VehicleMode;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/*
MANUAL:     base_mode=29 custom_mode=50593792
ACRO:       base_mode=65 custom_mode=327680
STABILIZED: base_mode=81 custom_mode=458752
ALTCTL:     base_mode=81 custom_mode=131072
POSCTL:     base_mode=81 custom_mode=196608
MISSION:    base_mode=29 custom_mode=67371008
LOITER:     base_mode=29 custom_mode=50593792
RTL:        base_mode=29 custom_mode=84148224
TAKEOFF:    base_mode=29 custom_mode=33816576
LAND:       base_mode=29 custom_mode=100925440

base_mode is 157 when armed and 29 when disarmed. Manual/Stabilized uses base_mode 81 for some reason.
Best to just use the current base_mode when setting modes. That seems to work.

Overall, this is a dumb way to deal with PX4 modes. QGC uses a C union type to determine mode,
and it works well. Also unions only exist in C and C++, so there's that.
 */

public enum Px4Mode implements BaseMode<Px4Mode> {
    MANUAL("Manual",
            PX4MavBaseMode.STABILIZED,
            Px4MavCustomMode.MANUAL,
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_MANUAL,
            0, true, true, true),
    STABILIZED("Stabilized",
            PX4MavBaseMode.STABILIZED,
            Px4MavCustomMode.STABILIZED,
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_STABILIZED,
            0, true, true, true),
    ACRO("Acro",
            PX4MavBaseMode.ACRO,
            Px4MavCustomMode.ACRO,
            Px4Util.CUSTOM_ENABLED | Px4Util.RAW_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_ACRO,
            0, true, true, true),
    RATTITUDE("Rattitude",
            PX4MavBaseMode.NONE,
            0L,
            Px4Util.CUSTOM_ENABLED | Px4Util.RAW_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_RATTITUDE,
            0, true, true, true),
    ALTCTL("ALTCTL",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.ALTCTL,
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_ALTCTL,
            0, true, true, true),
    POSCTL("POSCTL",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.POSCTL,
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_POSCTL,
            0, true, true, true),
    LOITER("Loiter",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.LOITER,
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_LOITER, true, true, true),
    MISSION("Mission",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.MISSION,
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_MISSION, true, true, true),
    RTL("RTL",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.RTL,
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_RTL, true, true, true),
    FOLLOW_ME("Follow Me",
            PX4MavBaseMode.NONE,
            0L,
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET, true, false, true),
    OFFBOARD("Offboard",
            PX4MavBaseMode.NONE,
            0L,
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, true, false, true),
    LAND("Land",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.LAND,
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_LAND, false, true, true),
    READY("Ready",
            PX4MavBaseMode.NONE,
        0,
        Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_READY, false, true, true),
    RTGS("RTGS",
            PX4MavBaseMode.NONE,
            0L,
        Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_RTGS, false, true, true),
    TAKEOFF("Takeoff",
            PX4MavBaseMode.NONE,
            Px4MavCustomMode.TAKEOFF,
        Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF, false, true, true),
    UNKNOWN("Unknown",
            PX4MavBaseMode.NONE,
            0L,
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            0, 0, false, false, false)
    ;

    private static final String TAG = Px4Mode.class.getSimpleName();

    private final int mainMode;
    private final int customMode;
    private final int customSubMode;
    private final long mavMainMode;
    private final long mavCustomMode;
    private final boolean canBeSet;
    private final boolean fixedWingCompat;
    private final boolean multiRotorCompat;

    private final String name;

    Px4Mode(String name, long mavMainMode, long mavCustomMode, int mainMode, int customMode, int customSubMode, boolean canSet, boolean fwCompat, boolean mcCompat) {
        this.name = name;
        this.mavMainMode = mavMainMode;
        this.mavCustomMode = mavCustomMode;
        this.mainMode = mainMode;
        this.customMode = customMode;
        this.customSubMode = customSubMode;
        this.canBeSet = canSet;
        this.fixedWingCompat = fwCompat;
        this.multiRotorCompat = mcCompat;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Px4Mode getNativeMode() {
        return this;
    }

    public boolean hasMavBaseMode() { return mavMainMode != PX4MavBaseMode.NONE; }

    public int getMainMode() {
        return mainMode;
    }

    public int getCustomMode() {
        return customMode;
    }

    public int getCustomSubMode() {
        return customSubMode;
    }

    public boolean canBeSet() { return canBeSet; }
    public boolean isFixedWing() { return fixedWingCompat; }
    public boolean isMultiRotor() { return multiRotorCompat; }

    public long getModeChangeValue() {
        final ByteBuffer buf = ByteBuffer.allocate(8);
        buf.order(ByteOrder.BIG_ENDIAN);
        buf.putInt(0);
        buf.putShort((short)customMode);
        buf.putShort((short)customSubMode);
        buf.rewind();
        return buf.getLong();
    }

    public static List<VehicleMode> getUserModesForType(int type) {
        final List<VehicleMode> modes = new ArrayList<>();

        for(Px4Mode p: userModesForType(type)) {
            final VehicleMode vm = toVehicleMode(p, type);
            if(vm != null && vm != VehicleMode.UNKNOWN) {
                modes.add(vm);
            } else {
//                Log.v(TAG, String.format("No mode found for %s", p));
            }
        }

        return modes;
    }

    public static Px4Mode[] userModesForType(int type) {
        final List<Px4Mode> list = new ArrayList<>();

        if(isCopter(type)) {
            list.add(Px4Mode.STABILIZED);
            list.add(Px4Mode.LOITER);
            list.add(Px4Mode.ALTCTL);
            list.add(Px4Mode.POSCTL);
            list.add(Px4Mode.MISSION);
            list.add(Px4Mode.OFFBOARD);
            list.add(Px4Mode.ACRO);
            list.add(Px4Mode.RTL);
//            for(Px4Mode m: values()) {
//                if(m.canBeSet && m.multiRotorCompat) {
//                    list.add(m);
//                }
//            }
        } else if(isPlane(type)) {
            for(Px4Mode m: values()) {
                if(m.canBeSet && m.fixedWingCompat) {
                    list.add(m);
                }
            }
        } else {
            // TODO: Need rover types
            list.add(Px4Mode.UNKNOWN);
        }

        return list.toArray(new Px4Mode[list.size()]);
    }

    public static Px4Mode getPx4Mode(VehicleMode mode, int type) {
        if(isCopter(type)) {
            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        }

        return Px4Util.toPx4Mode(mode, type);
    }

    public static VehicleMode toVehicleMode(Px4Mode mode, int type) {
        if(isCopter(type)) {
            return Px4Util.toCopterMode(mode);
        } else if(isPlane(type)) {
            return Px4Util.toPlaneMode(mode);
        } else {
            return VehicleMode.UNKNOWN;
        }
    }

    public static Px4Mode getHackyAssMode(msg_heartbeat msg) {
        for(Px4Mode mode: values()) {
            boolean baseMatch = (mode.hasMavBaseMode())?
                (mode.mavMainMode == msg.base_mode): true;

            if(mode.mavCustomMode == msg.custom_mode && baseMatch) {
                switch(msg.type) {
                    case MAV_TYPE.MAV_TYPE_FIXED_WING: {
                        if(mode.isFixedWing()) return mode;
                    }
                    
                    case MAV_TYPE.MAV_TYPE_QUADROTOR: {
                        if(mode.isMultiRotor()) return mode;
                    }
                    
                    case MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR:
                    case MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR:
                    case MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR:
                    case MAV_TYPE.MAV_TYPE_VTOL_RESERVED2:
                    case MAV_TYPE.MAV_TYPE_VTOL_RESERVED3:
                    case MAV_TYPE.MAV_TYPE_VTOL_RESERVED4:
                    case MAV_TYPE.MAV_TYPE_VTOL_RESERVED5: {
                        return mode;
                    }
                }
            }
        }

        return Px4Mode.UNKNOWN;
    }

    public static Px4Mode getHeartbeatMode(int baseMode, long customMode, int type) {
        for(Px4Mode mode: values()) {
            if(mode.mainMode == baseMode) {
                if(mode.customMode == customMode) {
                    return mode;
                }
            }
        }

        return Px4Mode.UNKNOWN;
    }

    public static boolean isValid(Px4Mode mode) {
        return (mode != null && mode != UNKNOWN);
    }

    public static boolean isCopter(int type) {
        switch (type) {
            case MAV_TYPE.MAV_TYPE_TRICOPTER:
            case MAV_TYPE.MAV_TYPE_QUADROTOR:
            case MAV_TYPE.MAV_TYPE_HEXAROTOR:
            case MAV_TYPE.MAV_TYPE_OCTOROTOR:
            case MAV_TYPE.MAV_TYPE_HELICOPTER:
                return true;

            default:
                return false;
        }
    }

    public static boolean isPlane(int type) {
        switch(type) {
            case MAV_TYPE.MAV_TYPE_FIXED_WING:
            case MAV_TYPE.MAV_TYPE_FLAPPING_WING:
                return true;

            default:
                return false;
        }
    }
}
