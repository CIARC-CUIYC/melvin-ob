#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Upstream {
    #[prost(message, optional, tag = "1")]
    pub ping: Option<Ping>,
}

#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Ping {
    #[prost(string, optional, tag = "1")]
    pub echo: Option<String>,
}
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Downstream {
    #[prost(oneof = "Content", tags = "1, 2, 3")]
    pub content: Option<Content>,
}
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Pong {
    #[prost(string, optional, tag = "1")]
    pub echo: Option<String>,
}
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Image {
    #[prost(int32, tag = "1")]
    pub width: i32,
    #[prost(int32, tag = "2")]
    pub height: i32,
    #[prost(int32, tag = "3")]
    pub offset_x: i32,
    #[prost(int32, tag = "4")]
    pub offset_y: i32,
    #[prost(bytes = "vec", tag = "5")]
    pub data: Vec<u8>,
}
#[derive(Clone, Copy, PartialEq, ::prost::Message)]
pub struct Telemetry {
    #[prost(int64, tag = "1")]
    pub timestamp: i64,
    #[prost(enumeration = "SatelliteState", tag = "2")]
    pub state: i32,
    #[prost(int32, tag = "3")]
    pub position_x: i32,
    #[prost(int32, tag = "4")]
    pub position_y: i32,
    #[prost(float, tag = "5")]
    pub velocity_x: f32,
    #[prost(float, tag = "6")]
    pub velocity_y: f32,
    #[prost(float, tag = "7")]
    pub battery: f32,
    #[prost(float, tag = "8")]
    pub fuel: f32,
    #[prost(int32, tag = "9")]
    pub data_sent: i32,
    #[prost(int32, tag = "10")]
    pub data_received: i32,
    #[prost(float, tag = "11")]
    pub distance_covered: f32,
}
#[derive(Clone, PartialEq, ::prost::Oneof)]
pub enum Content {
    #[prost(message, tag = "1")]
    Pong(Pong),
    #[prost(message, tag = "2")]
    Image(Image),
    #[prost(message, tag = "3")]
    Telemetry(Telemetry),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, ::prost::Enumeration)]
#[repr(i32)]
pub enum SatelliteState {
    None = 0,
    Deployment = 1,
    Safe = 2,
    Communication = 3,
    Charge = 4,
    Acquisition = 5,
    Transition = 6,
}
impl SatelliteState {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Deployment => "deployment",
            Self::Safe => "safe",
            Self::Communication => "communication",
            Self::Charge => "charge",
            Self::Acquisition => "acquisition",
            Self::Transition => "transition",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "none" => Some(Self::None),
            "deployment" => Some(Self::Deployment),
            "safe" => Some(Self::Safe),
            "communication" => Some(Self::Communication),
            "charge" => Some(Self::Charge),
            "acquisition" => Some(Self::Acquisition),
            "transition" => Some(Self::Transition),
            _ => None,
        }
    }
}
