package com.o3dr.services.android.lib.drone.property

import android.content.Context
import com.o3dr.android.client.R
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import android.os.Parcel
import android.os.Parcelable
import com.MAVLink.common.msg_adsb_vehicle
import com.MAVLink.enums.ADSB_EMITTER_TYPE

/**
 * An ADSB vehicle
 */
class ADSBVehicle : DroneAttribute {
    enum class Type(val emitterType: Int, private val resId: Int) {
        Unknown(0, 0),
        Light(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT, R.string.adsb_t_light),
        Small(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL, R.string.adsb_t_small),
        Large(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE, R.string.adsb_t_large),
        HiVortexLarge(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE, R.string.adsb_t_hv_large),
        Heavy(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY, R.string.adsb_t_heavy),
        HighlyManuv(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV, R.string.adsb_t_highly_manuv),
        RotoCraft(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT, R.string.adsb_t_rotocraft),
        Unassigned(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED, R.string.adsb_t_unassigned),
        Glider(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER, R.string.adsb_t_glider),
        LighterThanAir(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR, R.string.adsb_t_lighter_air),
        Parachute(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE, R.string.adsb_t_parachute),
        Ultralight(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT, R.string.adsb_t_ultralight),
        Unassigned2(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2, R.string.adsb_t_unassigned2),
        UAV(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV, R.string.adsb_t_uav),
        Space(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE, R.string.adsb_t_space),
        Unassigned3(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3, R.string.adsb_t_unassigned3),
        EmergencySurface(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE, R.string.adsb_t_emergency_surface),
        ServiceSurface(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE, R.string.adsb_t_service_surface),
        PointObstacle(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE, R.string.adsb_t_point_obstacle)
        ;

        fun getLabel(context: Context): String {
            return if (resId == 0) "Unknown" else context.getString(resId)
        }

        companion object {
            fun fromId(id: Int): Type {
                for (t in values()) {
                    if (t.emitterType == id) {
                        return t
                    }
                }
                return Unknown
            }
        }
    }

    var icaoAddress: Long = 0
        private set

    var coord: LatLongAlt? = null
        private set

    var heading = 0.0
        private set

    var horizVelocity // meters/second
            = 0.0
        private set

    var vertVelocity // meters/second
            = 0.0
        private set

    var squawk = 0
        private set

    var flags // ADSB_FLAGS
            = 0
        private set

    var altitudeType // ADSB_ALTITUDE_TYPE
            = 0
        private set

    var emitterType // ADSB_EMITTER_TYPE
            = 0
        private set

    var tslc = 0
        private set

    var callSign: String? = null
        private set

    var type = Type.Unknown
        private set

    constructor() : super() {}

    override fun toString(): String {
        return "ADSBVehicle{" +
                "icaoAddress=" + icaoAddress +
                ", coord=" + coord +
                ", heading=" + heading +
                ", horizVelocity=" + horizVelocity +
                ", vertVelocity=" + vertVelocity +
                ", squawk=" + squawk +
                ", flags=" + flags +
                ", altitudeType=" + altitudeType +
                ", emitterType=" + emitterType +
                ", tslc=" + tslc +
                ", type=" + type +
                ", callSign='" + callSign + '\'' +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeLong(icaoAddress)
        dest.writeParcelable(coord, 0)
        dest.writeDouble(heading)
        dest.writeDouble(horizVelocity)
        dest.writeDouble(vertVelocity)
        dest.writeInt(squawk)
        dest.writeInt(flags)
        dest.writeInt(altitudeType)
        dest.writeInt(emitterType)
        dest.writeInt(tslc)
        dest.writeString(callSign)
        dest.writeInt(type.ordinal)
    }

    private constructor(input: Parcel) {
        icaoAddress = input.readLong()
        coord = input.readParcelable(LatLongAlt::class.java.classLoader)
        heading = input.readDouble()
        horizVelocity = input.readDouble()
        vertVelocity = input.readDouble()
        squawk = input.readInt()
        flags = input.readInt()
        altitudeType = input.readInt()
        emitterType = input.readInt()
        tslc = input.readInt()
        callSign = input.readString()
        type = Type.fromId(input.readInt())
    }

    companion object {
        @JvmStatic
        fun populate(v: ADSBVehicle, msg: msg_adsb_vehicle): ADSBVehicle {
            v.icaoAddress = msg.ICAO_address
            val newLat = msg.lat / 1E7
            val newLong = msg.lon / 1E7
            val altMeters = (msg.altitude / 1000).toDouble() // mm->m
            v.coord = LatLongAlt(newLat, newLong, altMeters)
            v.heading = (msg.heading / 100).toDouble()
            v.horizVelocity = (msg.hor_velocity / 100).toDouble() // cm/s -> m/s
            v.vertVelocity = (msg.ver_velocity / 100).toDouble() // cm/s -> m/s
            v.flags = msg.flags
            v.squawk = msg.squawk
            v.altitudeType = msg.altitude_type.toInt()
            v.callSign = msg.getCallsign()
            v.emitterType = msg.emitter_type.toInt()
            v.tslc = msg.tslc.toInt()
            v.type = Type.fromId(msg.emitter_type.toInt())
            return v
        }

        @JvmField
        val CREATOR: Parcelable.Creator<ADSBVehicle> = object : Parcelable.Creator<ADSBVehicle> {
            override fun createFromParcel(source: Parcel): ADSBVehicle? {
                return ADSBVehicle(source)
            }

            override fun newArray(size: Int): Array<ADSBVehicle?> {
                return arrayOfNulls(size)
            }
        }
    }
}
