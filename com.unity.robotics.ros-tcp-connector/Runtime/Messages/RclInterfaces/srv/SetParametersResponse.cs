//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RclInterfaces
{
    [Serializable]
    public class SetParametersResponse : Message
    {
        public const string k_RosMessageName = "rcl_interfaces/SetParameters";
        public override string RosMessageName => k_RosMessageName;

        //  Indicates whether setting each parameter succeeded or not and why.
        public SetParametersResultMsg[] results;

        public SetParametersResponse()
        {
            this.results = new SetParametersResultMsg[0];
        }

        public SetParametersResponse(SetParametersResultMsg[] results)
        {
            this.results = results;
        }

        public static SetParametersResponse Deserialize(MessageDeserializer deserializer) => new SetParametersResponse(deserializer);

        private SetParametersResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.results, SetParametersResultMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.results);
            serializer.Write(this.results);
        }

        public override string ToString()
        {
            return "SetParametersResponse: " +
            "\nresults: " + System.String.Join(", ", results.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
