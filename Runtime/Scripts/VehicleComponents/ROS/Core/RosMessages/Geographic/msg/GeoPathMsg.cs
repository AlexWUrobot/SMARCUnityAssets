//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geographic
{
    [Serializable]
    public class GeoPathMsg : Message
    {
        public const string k_RosMessageName = "geographic_msgs/GeoPath";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public GeoPoseStampedMsg[] poses;

        public GeoPathMsg()
        {
            this.header = new Std.HeaderMsg();
            this.poses = new GeoPoseStampedMsg[0];
        }

        public GeoPathMsg(Std.HeaderMsg header, GeoPoseStampedMsg[] poses)
        {
            this.header = header;
            this.poses = poses;
        }

        public static GeoPathMsg Deserialize(MessageDeserializer deserializer) => new GeoPathMsg(deserializer);

        private GeoPathMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.poses, GeoPoseStampedMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.WriteLength(this.poses);
            serializer.Write(this.poses);
        }

        public override string ToString()
        {
            return "GeoPathMsg: " +
            "\nheader: " + header.ToString() +
            "\nposes: " + System.String.Join(", ", poses.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}