/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE PROCESSING_IMAGE PACKING
package com.MAVLink.sui_custom;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
* Identify the image being processed, by index.
*/
public class msg_processing_image extends MAVLinkMessage{

    public static final int MAVLINK_MSG_ID_PROCESSING_IMAGE = 15;
    public static final int MAVLINK_MSG_LENGTH = 6;
    private static final long serialVersionUID = MAVLINK_MSG_ID_PROCESSING_IMAGE;


      
    /**
    * index of image being processed
    */
    public int index;
      
    /**
    * total number of images
    */
    public int total;
      
    /**
    * System ID
    */
    public short target_system;
      
    /**
    * Component ID
    */
    public short target_component;
    

    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_PROCESSING_IMAGE;
              
        packet.payload.putUnsignedShort(index);
              
        packet.payload.putUnsignedShort(total);
              
        packet.payload.putUnsignedByte(target_system);
              
        packet.payload.putUnsignedByte(target_component);
        
        return packet;
    }

    /**
    * Decode a processing_image message into this class fields
    *
    * @param payload The message to decode
    */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
              
        this.index = payload.getUnsignedShort();
              
        this.total = payload.getUnsignedShort();
              
        this.target_system = payload.getUnsignedByte();
              
        this.target_component = payload.getUnsignedByte();
        
    }

    /**
    * Constructor for a new message, just initializes the msgid
    */
    public msg_processing_image(){
        msgid = MAVLINK_MSG_ID_PROCESSING_IMAGE;
    }

    /**
    * Constructor for a new message, initializes the message with the payload
    * from a mavlink packet
    *
    */
    public msg_processing_image(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_PROCESSING_IMAGE;
        unpack(mavLinkPacket.payload);        
    }

            
    /**
    * Returns a string with the MSG name and data
    */
    public String toString(){
        return "MAVLINK_MSG_ID_PROCESSING_IMAGE - sysid:"+sysid+" compid:"+compid+" index:"+index+" total:"+total+" target_system:"+target_system+" target_component:"+target_component+"";
    }
}
        