use crate::flight_control::camera_controller::EncodedImageExtract;

#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Upstream {
    #[prost(oneof = "UpstreamContent", tags = "1, 2, 3, 4, 5, 6")]
    pub content: Option<UpstreamContent>,
}

#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Ping {
    #[prost(string, optional, tag = "1")]
    pub echo: Option<String>,
}
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Downstream {
    #[prost(oneof = "DownstreamContent", tags = "1, 2, 3, 4")]
    pub content: Option<DownstreamContent>,
}
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Pong {
    #[prost(string, optional, tag = "1")]
    pub echo: Option<String>,
}
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Image {
    #[prost(uint32, tag = "1")]
    pub width: u32,
    #[prost(uint32, tag = "2")]
    pub height: u32,
    #[prost(uint32, tag = "3")]
    pub offset_x: u32,
    #[prost(uint32, tag = "4")]
    pub offset_y: u32,
    #[prost(bytes = "vec", tag = "5")]
    pub data: Vec<u8>,
}

impl Image {
    pub(crate) fn from_encoded_image_extract(encoded_image: EncodedImageExtract) -> Self {
        Self {
            height: encoded_image.size.y(),
            width: encoded_image.size.x(),
            offset_x: encoded_image.offset.x(),
            offset_y: encoded_image.offset.y(),
            data: encoded_image.data,
        }
    }
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
    #[prost(uint32, tag = "9")]
    pub data_sent: u32,
    #[prost(uint32, tag = "10")]
    pub data_received: u32,
    #[prost(float, tag = "11")]
    pub distance_covered: f32,
}

#[derive(Clone, Copy, PartialEq, prost::Message)]
pub struct SubmitResponse {
    #[prost(bool, tag = "1")]
    pub success: bool,
    #[prost(uint32, optional, tag = "2")]
    pub objective_id: Option<u32>,
}

#[derive(Clone, PartialEq, prost::Oneof)]
pub enum DownstreamContent {
    #[prost(message, tag = "1")]
    Pong(Pong),
    #[prost(message, tag = "2")]
    Image(Image),
    #[prost(message, tag = "3")]
    Telemetry(Telemetry),
    #[prost(message, tag = "4")]
    SubmitResponse(SubmitResponse),
}

#[derive(Clone, PartialEq, prost::Oneof)]
pub enum UpstreamContent {
    #[prost(message, tag = "1")]
    Ping(Ping),
    #[prost(message, tag = "2")]
    GetFullImage(GetFullImage),
    #[prost(message, tag = "3")]
    GetSnapshotDiffImage(GetSnapshotDiffImage),
    #[prost(message, tag = "4")]
    CreateSnapshotImage(CreateSnapshotImage),
    #[prost(message, tag = "5")]
    SubmitObjective(SubmitObjective),
    #[prost(message, tag = "6")]
    SubmitDailyMap(SubmitDailyMap),
}
#[derive(Clone, Copy, PartialEq, prost::Message)]
pub struct GetFullImage {}

#[derive(Clone, Copy, PartialEq, prost::Message)]
pub struct SubmitObjective {
    #[prost(uint32, tag = "1")]
    pub objective_id: u32,
    #[prost(uint32, tag = "2")]
    pub width: u32,
    #[prost(uint32, tag = "3")]
    pub height: u32,
    #[prost(uint32, tag = "4")]
    pub offset_x: u32,
    #[prost(uint32, tag = "5")]
    pub offset_y: u32,
}

#[derive(Clone, Copy, PartialEq, prost::Message)]
pub struct SubmitDailyMap {}

#[derive(Clone, Copy, PartialEq, prost::Message)]
pub struct GetSnapshotDiffImage {}

#[derive(Clone, Copy, PartialEq, prost::Message)]
pub struct CreateSnapshotImage {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
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
    /// String value of the enum field names used in the `ProtoBuf` definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the `ProtoBuf` definition does not change) and safe for programmatic use.
    pub fn as_str_name(self) -> &'static str {
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
    /// Creates an enum from field names used in the `ProtoBuf` definition.
    pub fn from_str_name(value: &str) -> Option<Self> {
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
