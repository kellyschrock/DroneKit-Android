package com.o3dr.services.android.lib.drone.mission.item.command;

import android.os.Parcel;

import com.o3dr.services.android.lib.drone.mission.MissionItemType;
import com.o3dr.services.android.lib.drone.mission.item.MissionItem;

import java.util.Objects;

/**
 * Created by fhuya on 11/10/14.
 */
public class DelayCondition extends MissionItem implements MissionItem.Command, android.os.Parcelable {

    private double seconds = 0;

    public DelayCondition(){
        super(MissionItemType.DELAY_CONDITION);
    }

    public DelayCondition(DelayCondition copy){
        this();
        seconds = copy.seconds;
    }

    public double getSeconds() { return seconds; }
    public void setSeconds(double s) { seconds = s; }

    @Override
    public String toString() {
        return "DelayCondition{" +
            "seconds=" + seconds +
            '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        DelayCondition that = (DelayCondition) o;
        return Double.compare(that.seconds, seconds) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), seconds);
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        super.writeToParcel(dest, flags);
        dest.writeDouble(this.seconds);
    }

    private DelayCondition(Parcel in) {
        super(in);
        this.seconds = in.readDouble();
    }

    @Override
    public MissionItem clone() {
        return new DelayCondition(this);
    }

    public static final Creator<DelayCondition> CREATOR = new Creator<DelayCondition>() {
        public DelayCondition createFromParcel(Parcel source) {
            return new DelayCondition(source);
        }

        public DelayCondition[] newArray(int size) {
            return new DelayCondition[size];
        }
    };
}
