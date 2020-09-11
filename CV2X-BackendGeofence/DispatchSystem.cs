// <auto-generated>
//     Generated by the protocol buffer compiler.  DO NOT EDIT!
//     source: DispatchSystem.proto
// </auto-generated>
#pragma warning disable 1591, 0612, 3021
#region Designer generated code

using pb = global::Google.Protobuf;
using pbc = global::Google.Protobuf.Collections;
using pbr = global::Google.Protobuf.Reflection;
using scg = global::System.Collections.Generic;
/// <summary>Holder for reflection information generated from DispatchSystem.proto</summary>
public static partial class DispatchSystemReflection {

  #region Descriptor
  /// <summary>File descriptor for DispatchSystem.proto</summary>
  public static pbr::FileDescriptor Descriptor {
    get { return descriptor; }
  }
  private static pbr::FileDescriptor descriptor;

  static DispatchSystemReflection() {
    byte[] descriptorData = global::System.Convert.FromBase64String(
        string.Concat(
          "ChREaXNwYXRjaFN5c3RlbS5wcm90byKrAgoMRGlzcGF0Y2hEYXRhEhIKCmJ1",
          "c3N0b3BfaWQYASABKAkSEwoLYnVzc3RvcF9sYXQYAiABKAESEwoLYnVzc3Rv",
          "cF9sb24YAyABKAESEwoLem9uZV9yYWRpdXMYBCABKAUSGAoQdGltZV9pbnNp",
          "ZGVfem9uZRgFIAEoBRIZChF0aW1lX291dHNpZGVfem9uZRgGIAEoBRISCgpi",
          "dXNfbnVtYmVyGAcgASgFEhgKEGVzdF9hcnJpdmFsX3RpbWUYCCABKAQSGAoQ",
          "bnVtX29mX2NvbW11dGVycxgJIAEoBRIXCg90aW1lcl90aHJlc2hvbGQYCiAB",
          "KAUSGgoSY29tbXV0ZXJfdGhyZXNob2xkGAsgASgFEhYKDmlzX2luc2lkZV96",
          "b25lGAwgASgIQh8KCHByb3RvYnVmQhNEaXNwYXRjaFN5c3RlbVByb3RvYgZw",
          "cm90bzM="));
    descriptor = pbr::FileDescriptor.FromGeneratedCode(descriptorData,
        new pbr::FileDescriptor[] { },
        new pbr::GeneratedClrTypeInfo(null, null, new pbr::GeneratedClrTypeInfo[] {
          new pbr::GeneratedClrTypeInfo(typeof(global::DispatchData), global::DispatchData.Parser, new[]{ "BusstopId", "BusstopLat", "BusstopLon", "ZoneRadius", "TimeInsideZone", "TimeOutsideZone", "BusNumber", "EstArrivalTime", "NumOfCommuters", "TimerThreshold", "CommuterThreshold", "IsInsideZone" }, null, null, null, null)
        }));
  }
  #endregion

}
#region Messages
public sealed partial class DispatchData : pb::IMessage<DispatchData> {
  private static readonly pb::MessageParser<DispatchData> _parser = new pb::MessageParser<DispatchData>(() => new DispatchData());
  private pb::UnknownFieldSet _unknownFields;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public static pb::MessageParser<DispatchData> Parser { get { return _parser; } }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public static pbr::MessageDescriptor Descriptor {
    get { return global::DispatchSystemReflection.Descriptor.MessageTypes[0]; }
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  pbr::MessageDescriptor pb::IMessage.Descriptor {
    get { return Descriptor; }
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public DispatchData() {
    OnConstruction();
  }

  partial void OnConstruction();

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public DispatchData(DispatchData other) : this() {
    busstopId_ = other.busstopId_;
    busstopLat_ = other.busstopLat_;
    busstopLon_ = other.busstopLon_;
    zoneRadius_ = other.zoneRadius_;
    timeInsideZone_ = other.timeInsideZone_;
    timeOutsideZone_ = other.timeOutsideZone_;
    busNumber_ = other.busNumber_;
    estArrivalTime_ = other.estArrivalTime_;
    numOfCommuters_ = other.numOfCommuters_;
    timerThreshold_ = other.timerThreshold_;
    commuterThreshold_ = other.commuterThreshold_;
    isInsideZone_ = other.isInsideZone_;
    _unknownFields = pb::UnknownFieldSet.Clone(other._unknownFields);
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public DispatchData Clone() {
    return new DispatchData(this);
  }

  /// <summary>Field number for the "busstop_id" field.</summary>
  public const int BusstopIdFieldNumber = 1;
  private string busstopId_ = "";
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public string BusstopId {
    get { return busstopId_; }
    set {
      busstopId_ = pb::ProtoPreconditions.CheckNotNull(value, "value");
    }
  }

  /// <summary>Field number for the "busstop_lat" field.</summary>
  public const int BusstopLatFieldNumber = 2;
  private double busstopLat_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public double BusstopLat {
    get { return busstopLat_; }
    set {
      busstopLat_ = value;
    }
  }

  /// <summary>Field number for the "busstop_lon" field.</summary>
  public const int BusstopLonFieldNumber = 3;
  private double busstopLon_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public double BusstopLon {
    get { return busstopLon_; }
    set {
      busstopLon_ = value;
    }
  }

  /// <summary>Field number for the "zone_radius" field.</summary>
  public const int ZoneRadiusFieldNumber = 4;
  private int zoneRadius_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int ZoneRadius {
    get { return zoneRadius_; }
    set {
      zoneRadius_ = value;
    }
  }

  /// <summary>Field number for the "time_inside_zone" field.</summary>
  public const int TimeInsideZoneFieldNumber = 5;
  private int timeInsideZone_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int TimeInsideZone {
    get { return timeInsideZone_; }
    set {
      timeInsideZone_ = value;
    }
  }

  /// <summary>Field number for the "time_outside_zone" field.</summary>
  public const int TimeOutsideZoneFieldNumber = 6;
  private int timeOutsideZone_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int TimeOutsideZone {
    get { return timeOutsideZone_; }
    set {
      timeOutsideZone_ = value;
    }
  }

  /// <summary>Field number for the "bus_number" field.</summary>
  public const int BusNumberFieldNumber = 7;
  private int busNumber_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int BusNumber {
    get { return busNumber_; }
    set {
      busNumber_ = value;
    }
  }

  /// <summary>Field number for the "est_arrival_time" field.</summary>
  public const int EstArrivalTimeFieldNumber = 8;
  private ulong estArrivalTime_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public ulong EstArrivalTime {
    get { return estArrivalTime_; }
    set {
      estArrivalTime_ = value;
    }
  }

  /// <summary>Field number for the "num_of_commuters" field.</summary>
  public const int NumOfCommutersFieldNumber = 9;
  private int numOfCommuters_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int NumOfCommuters {
    get { return numOfCommuters_; }
    set {
      numOfCommuters_ = value;
    }
  }

  /// <summary>Field number for the "timer_threshold" field.</summary>
  public const int TimerThresholdFieldNumber = 10;
  private int timerThreshold_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int TimerThreshold {
    get { return timerThreshold_; }
    set {
      timerThreshold_ = value;
    }
  }

  /// <summary>Field number for the "commuter_threshold" field.</summary>
  public const int CommuterThresholdFieldNumber = 11;
  private int commuterThreshold_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int CommuterThreshold {
    get { return commuterThreshold_; }
    set {
      commuterThreshold_ = value;
    }
  }

  /// <summary>Field number for the "is_inside_zone" field.</summary>
  public const int IsInsideZoneFieldNumber = 12;
  private bool isInsideZone_;
  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public bool IsInsideZone {
    get { return isInsideZone_; }
    set {
      isInsideZone_ = value;
    }
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public override bool Equals(object other) {
    return Equals(other as DispatchData);
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public bool Equals(DispatchData other) {
    if (ReferenceEquals(other, null)) {
      return false;
    }
    if (ReferenceEquals(other, this)) {
      return true;
    }
    if (BusstopId != other.BusstopId) return false;
    if (!pbc::ProtobufEqualityComparers.BitwiseDoubleEqualityComparer.Equals(BusstopLat, other.BusstopLat)) return false;
    if (!pbc::ProtobufEqualityComparers.BitwiseDoubleEqualityComparer.Equals(BusstopLon, other.BusstopLon)) return false;
    if (ZoneRadius != other.ZoneRadius) return false;
    if (TimeInsideZone != other.TimeInsideZone) return false;
    if (TimeOutsideZone != other.TimeOutsideZone) return false;
    if (BusNumber != other.BusNumber) return false;
    if (EstArrivalTime != other.EstArrivalTime) return false;
    if (NumOfCommuters != other.NumOfCommuters) return false;
    if (TimerThreshold != other.TimerThreshold) return false;
    if (CommuterThreshold != other.CommuterThreshold) return false;
    if (IsInsideZone != other.IsInsideZone) return false;
    return Equals(_unknownFields, other._unknownFields);
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public override int GetHashCode() {
    int hash = 1;
    if (BusstopId.Length != 0) hash ^= BusstopId.GetHashCode();
    if (BusstopLat != 0D) hash ^= pbc::ProtobufEqualityComparers.BitwiseDoubleEqualityComparer.GetHashCode(BusstopLat);
    if (BusstopLon != 0D) hash ^= pbc::ProtobufEqualityComparers.BitwiseDoubleEqualityComparer.GetHashCode(BusstopLon);
    if (ZoneRadius != 0) hash ^= ZoneRadius.GetHashCode();
    if (TimeInsideZone != 0) hash ^= TimeInsideZone.GetHashCode();
    if (TimeOutsideZone != 0) hash ^= TimeOutsideZone.GetHashCode();
    if (BusNumber != 0) hash ^= BusNumber.GetHashCode();
    if (EstArrivalTime != 0UL) hash ^= EstArrivalTime.GetHashCode();
    if (NumOfCommuters != 0) hash ^= NumOfCommuters.GetHashCode();
    if (TimerThreshold != 0) hash ^= TimerThreshold.GetHashCode();
    if (CommuterThreshold != 0) hash ^= CommuterThreshold.GetHashCode();
    if (IsInsideZone != false) hash ^= IsInsideZone.GetHashCode();
    if (_unknownFields != null) {
      hash ^= _unknownFields.GetHashCode();
    }
    return hash;
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public override string ToString() {
    return pb::JsonFormatter.ToDiagnosticString(this);
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public void WriteTo(pb::CodedOutputStream output) {
    if (BusstopId.Length != 0) {
      output.WriteRawTag(10);
      output.WriteString(BusstopId);
    }
    if (BusstopLat != 0D) {
      output.WriteRawTag(17);
      output.WriteDouble(BusstopLat);
    }
    if (BusstopLon != 0D) {
      output.WriteRawTag(25);
      output.WriteDouble(BusstopLon);
    }
    if (ZoneRadius != 0) {
      output.WriteRawTag(32);
      output.WriteInt32(ZoneRadius);
    }
    if (TimeInsideZone != 0) {
      output.WriteRawTag(40);
      output.WriteInt32(TimeInsideZone);
    }
    if (TimeOutsideZone != 0) {
      output.WriteRawTag(48);
      output.WriteInt32(TimeOutsideZone);
    }
    if (BusNumber != 0) {
      output.WriteRawTag(56);
      output.WriteInt32(BusNumber);
    }
    if (EstArrivalTime != 0UL) {
      output.WriteRawTag(64);
      output.WriteUInt64(EstArrivalTime);
    }
    if (NumOfCommuters != 0) {
      output.WriteRawTag(72);
      output.WriteInt32(NumOfCommuters);
    }
    if (TimerThreshold != 0) {
      output.WriteRawTag(80);
      output.WriteInt32(TimerThreshold);
    }
    if (CommuterThreshold != 0) {
      output.WriteRawTag(88);
      output.WriteInt32(CommuterThreshold);
    }
    if (IsInsideZone != false) {
      output.WriteRawTag(96);
      output.WriteBool(IsInsideZone);
    }
    if (_unknownFields != null) {
      _unknownFields.WriteTo(output);
    }
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public int CalculateSize() {
    int size = 0;
    if (BusstopId.Length != 0) {
      size += 1 + pb::CodedOutputStream.ComputeStringSize(BusstopId);
    }
    if (BusstopLat != 0D) {
      size += 1 + 8;
    }
    if (BusstopLon != 0D) {
      size += 1 + 8;
    }
    if (ZoneRadius != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(ZoneRadius);
    }
    if (TimeInsideZone != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(TimeInsideZone);
    }
    if (TimeOutsideZone != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(TimeOutsideZone);
    }
    if (BusNumber != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(BusNumber);
    }
    if (EstArrivalTime != 0UL) {
      size += 1 + pb::CodedOutputStream.ComputeUInt64Size(EstArrivalTime);
    }
    if (NumOfCommuters != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(NumOfCommuters);
    }
    if (TimerThreshold != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(TimerThreshold);
    }
    if (CommuterThreshold != 0) {
      size += 1 + pb::CodedOutputStream.ComputeInt32Size(CommuterThreshold);
    }
    if (IsInsideZone != false) {
      size += 1 + 1;
    }
    if (_unknownFields != null) {
      size += _unknownFields.CalculateSize();
    }
    return size;
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public void MergeFrom(DispatchData other) {
    if (other == null) {
      return;
    }
    if (other.BusstopId.Length != 0) {
      BusstopId = other.BusstopId;
    }
    if (other.BusstopLat != 0D) {
      BusstopLat = other.BusstopLat;
    }
    if (other.BusstopLon != 0D) {
      BusstopLon = other.BusstopLon;
    }
    if (other.ZoneRadius != 0) {
      ZoneRadius = other.ZoneRadius;
    }
    if (other.TimeInsideZone != 0) {
      TimeInsideZone = other.TimeInsideZone;
    }
    if (other.TimeOutsideZone != 0) {
      TimeOutsideZone = other.TimeOutsideZone;
    }
    if (other.BusNumber != 0) {
      BusNumber = other.BusNumber;
    }
    if (other.EstArrivalTime != 0UL) {
      EstArrivalTime = other.EstArrivalTime;
    }
    if (other.NumOfCommuters != 0) {
      NumOfCommuters = other.NumOfCommuters;
    }
    if (other.TimerThreshold != 0) {
      TimerThreshold = other.TimerThreshold;
    }
    if (other.CommuterThreshold != 0) {
      CommuterThreshold = other.CommuterThreshold;
    }
    if (other.IsInsideZone != false) {
      IsInsideZone = other.IsInsideZone;
    }
    _unknownFields = pb::UnknownFieldSet.MergeFrom(_unknownFields, other._unknownFields);
  }

  [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
  public void MergeFrom(pb::CodedInputStream input) {
    uint tag;
    while ((tag = input.ReadTag()) != 0) {
      switch(tag) {
        default:
          _unknownFields = pb::UnknownFieldSet.MergeFieldFrom(_unknownFields, input);
          break;
        case 10: {
          BusstopId = input.ReadString();
          break;
        }
        case 17: {
          BusstopLat = input.ReadDouble();
          break;
        }
        case 25: {
          BusstopLon = input.ReadDouble();
          break;
        }
        case 32: {
          ZoneRadius = input.ReadInt32();
          break;
        }
        case 40: {
          TimeInsideZone = input.ReadInt32();
          break;
        }
        case 48: {
          TimeOutsideZone = input.ReadInt32();
          break;
        }
        case 56: {
          BusNumber = input.ReadInt32();
          break;
        }
        case 64: {
          EstArrivalTime = input.ReadUInt64();
          break;
        }
        case 72: {
          NumOfCommuters = input.ReadInt32();
          break;
        }
        case 80: {
          TimerThreshold = input.ReadInt32();
          break;
        }
        case 88: {
          CommuterThreshold = input.ReadInt32();
          break;
        }
        case 96: {
          IsInsideZone = input.ReadBool();
          break;
        }
      }
    }
  }

}

#endregion


#endregion Designer generated code