//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RclInterfaces
{
    [Serializable]
    public class ParameterValueMsg : Message
    {
        public const string k_RosMessageName = "rcl_interfaces/ParameterValue";
        public override string RosMessageName => k_RosMessageName;

        //  Used to determine which of the next *_value fields are set.
        //  ParameterType.PARAMETER_NOT_SET indicates that the parameter was not set
        //  (if gotten) or is uninitialized.
        //  Values are enumerated in `ParameterType.msg`.
        //  The type of this parameter, which corresponds to the appropriate field below.
        public byte type;
        //  "Variant" style storage of the parameter value. Only the value corresponding
        //  the type field will have valid information.
        //  Boolean value, can be either true or false.
        public bool bool_value;
        //  Integer value ranging from -9,223,372,036,854,775,808 to
        //  9,223,372,036,854,775,807.
        public long integer_value;
        //  A double precision floating point value following IEEE 754.
        public double double_value;
        //  A textual value with no practical length limit.
        public string string_value;
        //  An array of bytes, used for non-textual information.
        public sbyte[] byte_array_value;
        //  An array of boolean values.
        public bool[] bool_array_value;
        //  An array of 64-bit integer values.
        public long[] integer_array_value;
        //  An array of 64-bit floating point values.
        public double[] double_array_value;
        //  An array of string values.
        public string[] string_array_value;

        public ParameterValueMsg()
        {
            this.type = 0;
            this.bool_value = false;
            this.integer_value = 0;
            this.double_value = 0.0;
            this.string_value = "";
            this.byte_array_value = new sbyte[0];
            this.bool_array_value = new bool[0];
            this.integer_array_value = new long[0];
            this.double_array_value = new double[0];
            this.string_array_value = new string[0];
        }

        public ParameterValueMsg(byte type, bool bool_value, long integer_value, double double_value, string string_value, sbyte[] byte_array_value, bool[] bool_array_value, long[] integer_array_value, double[] double_array_value, string[] string_array_value)
        {
            this.type = type;
            this.bool_value = bool_value;
            this.integer_value = integer_value;
            this.double_value = double_value;
            this.string_value = string_value;
            this.byte_array_value = byte_array_value;
            this.bool_array_value = bool_array_value;
            this.integer_array_value = integer_array_value;
            this.double_array_value = double_array_value;
            this.string_array_value = string_array_value;
        }

        public static ParameterValueMsg Deserialize(MessageDeserializer deserializer) => new ParameterValueMsg(deserializer);

        private ParameterValueMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.type);
            deserializer.Read(out this.bool_value);
            deserializer.Read(out this.integer_value);
            deserializer.Read(out this.double_value);
            deserializer.Read(out this.string_value);
            deserializer.Read(out this.byte_array_value, sizeof(sbyte), deserializer.ReadLength());
            deserializer.Read(out this.bool_array_value, sizeof(bool), deserializer.ReadLength());
            deserializer.Read(out this.integer_array_value, sizeof(long), deserializer.ReadLength());
            deserializer.Read(out this.double_array_value, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.string_array_value, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.type);
            serializer.Write(this.bool_value);
            serializer.Write(this.integer_value);
            serializer.Write(this.double_value);
            serializer.Write(this.string_value);
            serializer.WriteLength(this.byte_array_value);
            serializer.Write(this.byte_array_value);
            serializer.WriteLength(this.bool_array_value);
            serializer.Write(this.bool_array_value);
            serializer.WriteLength(this.integer_array_value);
            serializer.Write(this.integer_array_value);
            serializer.WriteLength(this.double_array_value);
            serializer.Write(this.double_array_value);
            serializer.WriteLength(this.string_array_value);
            serializer.Write(this.string_array_value);
        }

        public override string ToString()
        {
            return "ParameterValueMsg: " +
            "\ntype: " + type.ToString() +
            "\nbool_value: " + bool_value.ToString() +
            "\ninteger_value: " + integer_value.ToString() +
            "\ndouble_value: " + double_value.ToString() +
            "\nstring_value: " + string_value.ToString() +
            "\nbyte_array_value: " + System.String.Join(", ", byte_array_value.ToList()) +
            "\nbool_array_value: " + System.String.Join(", ", bool_array_value.ToList()) +
            "\ninteger_array_value: " + System.String.Join(", ", integer_array_value.ToList()) +
            "\ndouble_array_value: " + System.String.Join(", ", double_array_value.ToList()) +
            "\nstring_array_value: " + System.String.Join(", ", string_array_value.ToList());
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
