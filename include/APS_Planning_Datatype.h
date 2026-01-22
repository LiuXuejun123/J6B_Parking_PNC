//
// Created by lxj on 2026/1/19.
//
#ifndef J6B_PARKING_PNC_APS_PLANNING_DATATYPE_H
#define J6B_PARKING_PNC_APS_PLANNING_DATATYPE_H
#include <cstdint>
#include <array>
// Todo 数组需要改为array
namespace J6B_AD {
    typedef float float32_t;
    typedef double float64_t;

    namespace APS_Planning {
        // Time stamp structure for timestamp representation
        struct TimeStamp {
            uint32_t sec;
            uint32_t nsec;
        };

        // Data header with timestamp, sequence number and frame ID
        struct DataHeader {
            TimeStamp stamp;
            uint32_t seq;
            int8_t frameId[32];
        };

        // 3D point with covariance matrix (3x3)
        struct Point3FWithCovariance {
            float32_t x;
            float32_t y;
            float32_t z;
            float32_t covariance[9];
        };

        // Quaternion with covariance matrix (3x3)
        struct Quaternion4FWithCovariance {
            float32_t x;
            float32_t y;
            float32_t z;
            float32_t w;
            float32_t covariance[9];
        };

        // Location parking data with timestamp and pose
        struct LocationParking {
            int64_t sec;
            int64_t nsec;
            int64_t seq;
            Point3FWithCovariance position;
            Quaternion4FWithCovariance orientation;
        };

        // Obstacle bounding box
        struct ObstacleBox {
            float32_t targetHeading;
            float32_t height;
            float32_t width;
            float32_t length;
        };

        // 3D point (float)
        struct Point3F {
            float32_t x;
            float32_t y;
            float32_t z;
        };

        // Base obstacle detection result
        struct BaseObstacleResult {
            ObstacleBox bbox;
            Point3F position;
            Point3F relativePosition;
            Point3F positionVariance;
            float32_t headingAngleVariance;
            float32_t lifetime;
            float32_t confidence;
            int32_t id;
            bool traversable;
            int32_t sensorIndex;
            float32_t crashRiskTTC;
            Point3F polygon_bottom[4];
            Point3F polygon_top[4];
        };

        enum class VehicleType : uint8_t {
            VEHICLE_TYPE_UNKNOWN = 0,
            VEHICLE_TYPE_CAR = 1,
            VEHICLE_TYPE_TRUCK = 2,
            VEHICLE_TYPE_BUS = 3,
        };
        enum class MovingState : uint8_t {
            MS_UNKNOWN = 0,
            MS_STATIC = 1,
            MS_MOVING = 2,
        };

        enum class VehicleBlinkLightEnum : uint8_t {
            Vehicle_BLINKLIGHT_UNKNOWN = 0,
            Vehicle_BLINKLIGHT_OFF = 1,
            Vehicle_BLINKLIGHT_LEFT = 2,
            Vehicle_BLINKLIGHT_RIGHT = 3,
            Vehicle_BLINKLIGHT_BOTH = 4,
        };
        enum class VehicleBrakeLightEnum : uint8_t {
            Vehicle_BRAKELIGHT_UNKNOWN = 0,
            Vehicle_BRAKELIGHT_OFF = 1,
            Vehicle_BRAKELIGHT_ON = 2,
        };

        enum class VehicleBeamLightEnum : uint8_t {
            Vehicle_BEAMLIGHT_UNKNOWN = 0,
            Vehicle_BEAMLIGHT_OFF = 1,
            Vehicle_BEAMLIGHT_LOW = 2,
            Vehicle_BEAMLIGHT_HIGH = 3,
        };

        enum class VehicleDoorstateEnum : uint8_t {
            Vehicle_DOORSTATE_UNKNOWN = 0,
            Vehicle_DOORSTATE_OFF = 1,
            Vehicle_DOORSTATE_ON = 2,
        };

        enum class VehicleEgoLaneRelationEnum : uint8_t {
            Vehicle_Ego_Lane_Relation_Overlap = 0,
            Vehicle_Ego_Lane_Relation_No_Overlap = 1,
            Vehicle_Ego_Lane_Relation_Others = 2,
            Vehicle_Ego_Lane_Relation_Unknown = 3,
        };

        enum class VehicleCutinstateEnum : uint8_t {
            Vehicle_CUTINSTATE_UNKNOWN = 0,
            Vehicle_CUTINSTATE_OFF = 1,
            Vehicle_CUTINSTATE_ON = 2,
        };

        struct VehicleBlinkLightInfo {
            float blink_light_left_score;
            float blink_light_right_score;
            float blink_light_both_score;
            VehicleBlinkLightEnum blink_light_state;
        };

        struct VehicleBrakeLightInfo {
            float brake_light_on_score;
            VehicleBrakeLightEnum brake_light_state;
        };

        struct VehicleBeamLightInfo {
            float beam_light_on_state;
            VehicleBeamLightEnum beam_light_state;
        };

        struct VehicleDoorStateInfo {
            float door_state_on_score;
            VehicleDoorstateEnum door_state;
        };

        struct VehicleEgoLaneRelationInfo {
            float ego_lane_overlap_score;
            float ego_lane_no_overlap_score;
            VehicleEgoLaneRelationEnum ego_lane_relation;
        };

        struct VehicleCutinStateInfo {
            float cutin_state_on_score;
            VehicleCutinstateEnum cutin_state;
        };

        struct VehiclestateInfo {
            VehicleBlinkLightInfo blink_light_info;
            VehicleBrakeLightInfo brake_light_info;
            VehicleBeamLightInfo beam_light_info;
            VehicleDoorStateInfo door_state_info;
            VehicleEgoLaneRelationInfo ego_lane_relation_info;
            VehicleCutinStateInfo cutin_info;
        };

        // Vehicle detection result with motion and state information
        struct VehicleResult {
            BaseObstacleResult baseobstacleresult;
            Point3F velocity;
            Point3F relativeVelocity;
            Point3F velocityRelativeToGround;
            float32_t yawRate;
            float32_t relativeYawRate;
            Point3F acceleration;
            Point3F relativeAcceleration;
            Point3F accelerationRelativeToGround;
            Point3F velocityVariance;
            Point3F accelerationVariance;
            VehicleType classification;
            uint8_t subClassification;
            MovingState movingState;
            float staticConfidence;
            Point3F polygon[8];
            uint32_t polygonValidSize;
            VehiclestateInfo vehicleStateInfo;
        };

        // APA obstacles container
        struct APAObstacles {
            VehicleResult vehicles[64];
            uint8_t vehiclesValidSize;
        };
//APAObstacles
        enum class SlotType : uint8_t {
            Unknown         = 0,    // 0：未知类型
            HorizontalLine  = 1,    // 1：水平线库位
            VerticalLine    = 2,    // 2：垂直线库位
            DiagonalLine    = 3,    // 3：斜列线库位
            HorizontalSpace = 4,    // 4：水平空间库位
            VerticalSpace   = 5,    // 5：垂直空间库位
            Mechanical      = 6,    // 6：机械库位
            Custom          = 7     // 7：自定义库位
        };
        enum class SlotPositionType : uint8_t {
            SLOT_UNKNOWN_TYPE = 0,
            SLOT_LEFT_RAPA =1,
            SLOT_RIGHT_RAPA=2,
            SLOT_LEFT_VERT = 3,
            SLOT_RIGHT_VERT = 4,
            SLOT_LEFT_ANG_FORWARD=5,
            SLOT_LEFT_ANG_REVERSE=6,
            SLOT_RIGHT_ANG_FORWARD=7,
            SLOT_RIGHT_ANG_REVERSE=8,
        };

        // 2D point (float)
        struct Point2F {
            float x;
            float y;
        };

        // Quadrangle defined by four 2D points
        struct Quadrangle {
            Point2F point1;
            Point2F point2;
            Point2F point3;
            Point2F point4;
        };

        // Parking slot fusion result
        struct SlotFusionResult {
            int32_t id;
            uint32_t OccType;
            Point3F entrancePointA;
            Point3F entrancePointB;
            Point3F tailPointC;
            Point3F tailPointD;
            Point3F leftBump[4];
            uint8_t leftBumpHeight;
            uint8_t leftBumpValidSize;
            Point3F rightBump[4];
            uint8_t rightBumpHeight;
            uint8_t rightBumpValidSize;
            bool tailPointCReal;
            bool tailPointDReal;
            SlotPositionType slotPosition;
            SlotType slotType;
            uint8_t aOdType;
            uint8_t bOdType;
            Point3F uiA;
            Point3F uiB;
            Point3F uiC;
            Point3F uiD;
            uint8_t odTypeInSlot;
            uint8_t rePlanningFlag;
            uint8_t rearBestObserve;
            Quadrangle ramp[3];
            uint8_t sizeRamp;
            Quadrangle limitedBlock[3];
            uint8_t sizelimitedBlock;
            Quadrangle TailBar[3];
            uint8_t sizeTailBar;
            Point3F spacePoint[4];
            uint32_t fusionFlag;
            uint8_t isSelected;
            uint8_t customAdsorbType;
            uint8_t narrowSpot;
            uint8_t hasParkingSpace;
            uint8_t reserve[12];
        };

        // PLD fusion result container
        struct PldFusionResult {
            SlotFusionResult pldFusionResult[50];
            int32_t pldFusionResultVaildSize;
            int32_t selectValue;
        };

        // Line segment from start point to end point
        struct LineSptoEP {
            Point3F startPoint;
            Point3F endPoint;
            uint8_t type;
            uint8_t height;
            uint8_t rev;
        };

        // Line subset container
        struct LineSubset {
            LineSptoEP lineSubset[800];
            uint32_t lineSubsetValidSize;
        };

        // Freespace detection result
        struct FreespaceResult {
            DataHeader header;
            bool valid;
            uint8_t type;
            LineSubset freespaceBoundary;
        };
// Freespace
        // APA fusion output containing all sensor fusion data
        struct APAFusionOutput {
            DataHeader header;
            LocationParking location_data;
            APAObstacles obstacles;
            PldFusionResult pld;
            FreespaceResult freespace;
            uint8_t wallValidSize;
            uint8_t roadEdgeValidSize;
            uint8_t isDark;
            uint8_t existMps;
            uint8_t rev[8];
        };

        // Pose with position and orientation
        struct Pose {
            Point3FWithCovariance position;
            Quaternion4FWithCovariance orientation;
        };

        // Velocity with linear and angular components
        struct Velocity {
            Point3FWithCovariance linear;
            Point3FWithCovariance angular;
        };

        // Acceleration with linear and angular components
        struct Acceleration {
            Point3FWithCovariance linear;
            Point3FWithCovariance angular;
        };

        // Location data with pose, UTM coordinates, and motion information
        struct LocationData {
            DataHeader header;
            Pose pose;
            uint8_t utmLongitudeZone;
            uint8_t utmLatitudeZone;
            float32_t utmEastCoordinate;
            float32_t utmNorthCoordinate;
            float64_t longitude;
            float64_t latitude;
            float32_t altitude;
            float32_t yaw;
            float32_t pitch;
            float32_t roll;
            Velocity velocityLLA;
            Acceleration accelerationLLA;
            uint8_t loc_status;
            float32_t odometry;
            uint8_t locationState;
            uint8_t type;
            uint8_t resetCount;
        };
//LocationData
        // Parking UI to planning data interface
        struct ParkUIToPlanningData {
            DataHeader header;
            uint8_t moveDevRPAReqRPAOutModeSubT;
            uint8_t DrvrAsscSysParkMod;
            uint32_t selectedSlot;
            uint8_t ParkInOffsetChoice;
            uint8_t rspaMoveOtherDirection;
            uint8_t rspaVoiceControl;
            uint8_t FrntAndRePrkgInSwt;
            uint8_t ScreenOper;
            uint8_t PrkgSpdReq;
            uint8_t rev[16];
        };

        // Park state machine data
        struct ParkStateMachineData {
            DataHeader header;
            uint32_t tsmVersion;
            uint32_t	systemFault;
            bool	parkEnterReq;
            bool	parkExitReq;
            uint8_t	parkFunction;
            uint8_t	pdcCtrlRsn;
            uint8_t	tsmReqRpaOn;
            uint32_t	apaVersion;
            uint8_t	drvrAsscSysSts;
            uint8_t	lastPrkType;
            uint8_t	apaSubSysSts;
            uint8_t	apaStsFlag;
            uint8_t	engRunngReqByParkAssi1;
            uint8_t	parkInOrOut;
            uint8_t	lastParkgTypeConRq;
            float32_t	forwardDistance;
            float32_t	backwardDistance;
            float32_t	totalDistance;
            uint32_t	prkgFctTestPndReq;
            bool	prkgStopAudWarnReq;
            uint32_t	rpaVersion;
            uint8_t	prkgAssiSysRemPrkgSts;
            uint8_t	remStrSts;
            uint8_t	prkgModIncln;
            uint8_t	mobDevRPAReqResp;
            uint8_t	shakehandRq;
            uint8_t	prkgLockReq;
            uint8_t	vehPrkgLockTheftReq;
            uint8_t	vehWinSunRoofClsReq;
            uint32_t	AVMVersion;
            uint8_t	monrSysSts;
            uint8_t	obstclTrigResp;
            uint8_t	plaModStsResp;
            uint8_t	swtDispOnAndOffStsResp;
            uint8_t	thrDTouringViewResp;
            uint8_t	visnImgDispModResp;
            uint8_t	turnEntryAgWideVisResp;
            uint8_t	vehMdlClrResp;
            uint8_t	visnAgExtnResp;
            float32_t	threeDAngleReq;
            uint8_t	dispInterfaceChgReq;
            uint32_t	HMIVersion;
            uint8_t	drvrAsscSysDisp;
            uint8_t	prkgAuxInfoDisp;
            uint8_t	prkgBackDisp;
            uint8_t	prkgBtnStsDispGroupCancel;
            uint8_t	prkgBtnStsDispGroupPrkgIn;
            uint8_t	prkgBtnStsDispGroupPrkgOut;
            uint8_t	prkgFctDiDisp;
            uint8_t	prkgFctDiModResp;
            uint8_t	voicePromptReq;
            uint8_t	autActvPrkgFctSwtResp;
            uint8_t	autPrkgVoiceSwtResp;
            uint8_t	autPrkgVoiceTypResp;
            bool	imgSnSrCalSts;
            uint8_t	prkgDockDisReq;
            uint8_t	mobDevRPAStsUInt8;
            uint8_t	mobDevRPABtnStsToAppGrpPrkgInBtnSts;
            uint8_t	mobDevRPABtnStsToAppGrpPrkgOutBtnSts;
            uint8_t	prkgBtnStsDispset;
            uint8_t	prkgFctsSwtStsDisp;
            uint8_t	avmDrvrAsscSysDisp;
            uint8_t avmPrkgFctDiDisp;
            uint32_t	AMEVersion;
            uint8_t	APASelfCheck;
            uint8_t	APARecFault;
            uint8_t	APAUnRecFault_BefShakeHand;
            uint8_t	APAUnRecFault_Prepark;
            uint8_t	APASuspend;
            uint8_t	APAAbort;
            uint8_t	RPASelfCheck;
            uint8_t	RPARecFault;
            uint8_t	RPAUnRecFault;
            uint8_t	RPASuspend;
            uint8_t	RPAAbort;
            uint8_t	backPushAPAInfoReq;
            uint8_t	prkgProgsDisp;
            float32_t	prkgProcDstDisp;
            uint8_t	prkgOutModBtnStsToAPPPrkgOutModBtnSts1;
            float32_t	remStraightPrkgAssiSelnReqResp;
            uint32_t	apaSelfParkOrOutVersion;
            uint8_t	planFirstSucs;
            uint8_t	drvrAsscSysDispList;
            uint8_t	prkgOutModBtnStsToAPPPrkgOutModBtnSts5;
            uint8_t	prkgOutModBtnStsToAPPPrkgOutModBtnSts6;
            uint8_t	smAsyLatCtrlModReqGroupAsyLatCtrlModReq;
            uint8_t	smAsyALgtCtrlModAsyALgtCtrlMod1;
            uint8_t	LoSpdAndTrkBckSysSts;
            uint8_t	MAISts;
            uint8_t	PrkgBtnStsDispFloat;
            uint8_t	PrkgActivePush;
            uint8_t	NoTouchingForAVMReq;
            uint8_t	AutPrkgInBtnStsFromRPA;
            uint8_t	SMState;
            uint8_t	functionStatus[32];
            uint8_t	smfaultLevel[32];
            uint8_t	PrkgCameraStoreResp;
            uint8_t	rev[8];
            uint8_t	DPAModeBroadcast;
            uint16_t	smFctErrCode2HMI;
            uint8_t	FrntAndRePrkgInBtnSts;
            uint8_t	ViewLayoutReq;
            uint8_t	CustBtnSts;
            uint8_t	ExitCustBtnSts;
            uint8_t	smPrkgEmgyBrkSysSts;
            uint32_t	PEBVersion;
            uint8_t	RPABackAPAReq;
            uint8_t PrkgCaliBtnSts;
        };

        enum class EnumDriveMode : uint8_t {
            eDriveMode_Invalid  = 0,
            eDriveMode_Eco      = 1,
            eDriveMode_Comfort  = 2,
            eDriveMode_Sport    = 3,
            eDriveMode_Offroad  = 4,
        };

        // UI to planning data debug interface
        struct UIToPlanningDataDebug {
            DataHeader header;
            uint8_t moveDevRPAReqRPAOutModeSubT;
            uint8_t DrvrAsscSysParkMod;
            uint32_t selectedSlot;
            uint8_t ParkInOffsetChoice;
            float32_t driverSetSpeed;
            float32_t driverSetTimeGap;
            uint8_t overtakeAssistantRequest;
            EnumDriveMode DriveMode;
            uint8_t laneChangeRequest;
            uint8_t laneChangeStyle;
            uint8_t laneChangeEnable;
            uint8_t DPAButtonStatus;
            uint8_t rev[16];
        };

        // UI to STM data interface
        struct UIToSTMData {
            DataHeader header;
            uint8_t drvrAsscSysBtnPush;
            uint8_t	swtDispOnAndOffReq;
            uint8_t	navFrntActvSts;
            uint8_t	ihuFail;
            uint8_t	sceneModSeld;
            uint8_t	otherArPsdReq;
            uint8_t	mobDevRPAReqRSPACtrl;
            uint8_t	mobDevRPAReqRPAReq;
            uint8_t	mobDevRPAReqRPAOutModeSubT;
            uint8_t	mobDevRPAReqMobDevSts;
            uint8_t	remStraightPrkgAssiSelnReq;
            uint8_t	prkgTouchCoornReqTouchEveTyp;
            uint16_t	vehSpdIndcdVehSpdIndcd;
            uint8_t	smAutPrkgSlotNrReq;
            uint8_t	smPrkgInOrOutAndCncl;
            uint8_t	smPrkgIntrptReldBtn;
            uint8_t	smPrkgFctSwt;
            uint8_t	smAutActvPrkgFctSwt;
            uint8_t	fullScreenBtnPush;
            uint8_t	smSelPushApaInfo;
            uint8_t	smHpaSoftButnEnterRq;
            uint8_t	smMapBuildRq;
            uint8_t	smMapSaveRq;
            uint8_t	smMapSelectId;
            uint8_t	smDrvrAsscSysParkMod;
            uint8_t	MapBuildSts;
            uint8_t	AutValtPrkgMapSrcReq;
            uint8_t	AutValtPrkgL2BtnReqSettingFctPushBtnReq;
            uint8_t	AutValtPrkgL2BtnReqSettingNoSenceBtnReq;
            uint8_t	AutValtPrkgL2BtnReqSettingVoiceBtnReq;
            uint8_t	AutValtPrkgMapOperReqMapType;
            uint8_t	AutValtPrkgL1BtnReq;
            uint8_t	AutValtPrkgExperienceResp;
            uint8_t	LoSpdDrvgAssiShoPushResp;
            uint8_t	AutValtPrkgShoDstPushSwt;
            uint8_t	AutValtPrkgPictureUploadSwt;
            uint8_t	AutValtPrkgInsBtnReq;
            uint8_t	AutValtPrkgSwt;
            uint8_t	ProfPenSts1;
            uint8_t	AutValtPrkgMapOperReq2MapType2;
            uint32_t	AutValtPrkgMapOperReq2MapList1;
            uint32_t	AutValtPrkgMapOperReq2MapList2;
            uint32_t	AutValtPrkgMapOperReq2MapList3;
            uint32_t	AutValtPrkgMapOperReq2MapList4;
            uint8_t	AutValtPrkgL2PopBtnReq;
            uint8_t	VoiceActiveADCU;
            uint8_t	BtnTrSts1;
            uint8_t	BtnUnlckSts1;
            uint8_t	BtnLockSts1;
            uint8_t	BtnTrSts1KeyId;
            uint8_t	BtnUnlckSts1KeyId;
            uint8_t	BtnLockSts1KeyId;
            uint8_t	VoiceCtrlBrk;
            uint8_t	LoSpdDrvgAssiSwt;
            uint8_t	PrkgFrntOrReSelect;
            uint8_t	TrackingBackAssiSwt;
            uint8_t	PrkgEmgBrkSysSwt;
            uint8_t	AutPrkgVoiceSwt;
            uint8_t	LSDANotifPushReq;
            uint8_t	MonrSysSts;
            uint8_t	DrvrAsscWisdomBtn;
            uint8_t	ScenarioMod;
            uint8_t	AutPrkgVoiceTyp;
            uint8_t	MAISetting;
            uint8_t	handPrkgSlotNrReq;
            uint8_t	AsyAutDrvCtrlTypDIMReq;
            uint8_t	HmiDrvrSodReqPilot;
            uint8_t	DrvrCrsCtrlFctActvReq;
            uint8_t	DrvrACCFctDeactvnReq;
            uint8_t	DrvrACCrsSetSpdReq;
            float32_t	SetSpdForCrsCtrlFctActive;
            uint8_t	AccFusnTrfcReq;
            uint8_t	CrsCtrlTiGapAdjReq;
            uint8_t	VeSpdIndcdUnit;
            float32_t	VehSpdIndcd;
            uint8_t	DrvModReq;
            uint8_t	CamFltsStsFromDHU;
            uint8_t	DrvrDecStsFromDHU;
            uint8_t	EyeGazeZone;
            uint16_t	EyeGazeZoneTime;
            uint8_t	EyeOnRoadFromDHU;
            float32_t	eyeOpenFromDHUEyeOpenDegLe;
            float32_t	eyeOpenFromDHUEyeOpenDegRi;
            uint8_t	eyeOpenFromDHUEyeOpenLe;
            uint8_t	eyeOpenFromDHUEyeOpenRi;
            uint8_t	eyeOpenFromDHUEyeVisibleLe;
            uint8_t	eyeOpenFromDHUEyeVisibleRi;
            uint8_t	eyeOpenFromDHUFaceVisible;
            uint8_t	SnsrDrvrPfmncStsFromDHU;
            uint8_t	VoiceBrstMod;
            uint8_t	AutoLaneChgStyleSwSts;
            uint8_t	NOPCofmOfLanChagOnoff;
            uint8_t	HmiSodLanChgSwitch;
            uint8_t	HmiDrvrSodReqChg;
            uint8_t	AsyNoaUpgradeSwitch;
            uint8_t	SftyHmiEna;
            uint8_t	NOPRemindTypOfChgLaneReq;
            uint8_t	DrvrLaneChgAutActvSts;
            uint8_t	AutoLaneChgWithNaviSwOnoff;
            uint8_t	NavNOAStatusChangeReq;
            uint8_t	PrkgCameraStoreReq;
            uint8_t	VoiceCtrlPrk;
            uint8_t	DriftPrkgReq;
            uint8_t	HmiDrvrSodReqCnoaMainPage;
            uint8_t	HmiDrvrSodReqCnoaLrng;
            uint8_t	HmiDrvrSodReqFinishLrng;
            uint8_t	HmiCnoaCrsSwtOnOff;
            uint8_t	HmiCnoaSodLanChgSwt;
            uint8_t	HmiCnoaAutoLaneChgStyleSwSts;
            uint8_t HmiCnoaLrngSwtOnOff;
            uint8_t HmiDrvrSodReqContnsLrng;
        };

        // Bus to STM data interface
        struct BusToSTMData {
            DataHeader header;
            uint8_t trLockSts;
            uint8_t    doorDrvrLockSts;
            uint8_t    doorPassLockSts;
            uint8_t    doorLeReLockSts;
            uint8_t    doorRiReLockSts;
            uint8_t    brkSysStsBrkSysCapability;
            uint8_t   rev[2] ;
            uint8_t    calltypeAndStsEcallStatus;
            uint8_t    bltLockStAtDrvr;
            uint8_t    bltLockStErrStsAtDrvr;
            uint8_t    escStEscSt;
            uint8_t    prkLatLgtFailr;
            uint8_t    roadInclnQly;
            uint8_t    vehMtnSt;
            uint8_t    whlSpdCircumlFrntLeQf;
            uint8_t    whlSpdCircumlFrntRiQf;
            uint8_t    whlSpdCircumlReLeQf;
            uint8_t    whlSpdCircumlReRiQf;
            uint8_t   gearMov;
            uint8_t    accrPedlPsdAccrPedlPsd;
            uint8_t    accrPedlPsdSts;
            uint8_t    typExtReqToUpdQf;
            uint8_t    trsmParkLockd;
            uint8_t    doorDrvrSts;
            uint8_t    doorLeReSts;
            uint8_t    doorPassSts;
            uint8_t    doorRiReSts;
            uint8_t    hoodSts;
            uint8_t    mirrFoldStsAtDrvr;
            uint8_t    mirrFoldStsAtPass;
            uint8_t    trSts;
            uint8_t    trlrPrsnt;
            uint8_t    vehModMngtGlbSafe1CarModSts1;
            uint8_t    vehModMngtGlbSafe1UsgModSts;
            uint8_t    steerStsToParkAssi;
            uint8_t    indcrTypExtReq;
            uint8_t    rainfallAmnt;
            uint8_t    egyLvlElecMai;
            uint8_t    LeFrntTireMsgPWarnFlg;
            uint8_t    LeReTireMsgPWarnFlg;
            uint8_t    RiFrntTireMsgPWarnFlg;
            uint8_t    RiReTireMsgPWarnFlg;
            uint8_t    smLatShakeHandState;
            uint8_t    smLonShakeHandState;
            uint8_t    RCVCtrlReq;
            uint8_t    RCVSteertrimReq;
            uint8_t    MobDevRCVReq1MobDevSts;
            uint8_t    DcDcActvd;
            float32_t    SteerWhlSnsrAg;
            uint8_t    bleConSts;
            uint8_t    TwbrLockdPosn;
            uint8_t    FullElecTwbrPosn;
            uint8_t    VtsdModeSts;
            uint8_t    KeyReadStsToAsm[12];
            uint8_t    mobDevRPAReqRSPACtrl;
            uint8_t    mobDevRPAReqRPAReq;
            uint8_t    mobDevRPAReqRPAOutModeSubT;
            uint8_t    mobDevRPAReqMobDevSts;
            uint8_t    CrabMovModSts;
            uint8_t    TankTurnModSts;
            uint8_t    PassSeatSts2;
            uint8_t    HmiMaxEvMod;
            uint8_t    ADCUMaxEvMod;
            uint8_t    LampReqByVehHld;
            uint8_t    ADModCtrlInhbnADModCtrlInhbn;
            uint8_t    MsgReqByHillDwnCtrl;
            uint8_t    EscWarnIndcnReqEscWarnIndcnReq;
            uint8_t    AbsCtrlActvCtrlSts1;
            uint8_t    EngSt1WdStsEngSt1WdSts;
            uint8_t    CrsCtrlOvrdn;
            uint8_t    VehModMngtGlbSafe1PwrLvlElecMai;
            uint8_t    DrvrSteerWhlHldQly;
            uint8_t    DoorDrvrStsWithFacQlyDoorSts;
            uint8_t    DoorDrvrStsWithFacQlyFacQly;
            uint8_t    LatCtrlModCfmd;
            uint8_t    EscCtrlIndcn;
            uint8_t    PtDrvrSetgDrvModReq;
            uint8_t    FrontFourDRadarStsForDim;
            uint8_t    DrvrPrsntSts;
            uint8_t    WiprActv;
            uint8_t    StandStillMgrStsForHld;
            uint8_t    PtDrvrSetg;
            uint8_t    TPTFstatusError;
            uint8_t    TPTFstatus;
            uint8_t    TPTFTimeGap;
            uint8_t    HmiSodLanChgSwitch;
            uint8_t    HmiDrvrSodReqChg;
            uint8_t    TurnIndcrMonostable;
            uint8_t    DrvrLaneChgAutActvSts;
            uint8_t    AutoLaneChgWithNaviSwOnoff;
            uint8_t    NOPCofmOfLanChagOnoff;
            uint8_t    WipgSpdInfo;
            uint8_t    asySafeStopSts;
            uint8_t    HmiSupEndMod;
            uint8_t crashStsSafeSts;
        };

        // Version information
        struct VersionInfo {
            uint8_t moudle;
            uint32_t version;
        };

        // Car configuration entry
        struct CarConfig {
            uint16_t config;
            uint8_t value;
        };

        
        // Vehicle configuration
        struct VehicleConf {
            TimeStamp timeStamp;
            uint8_t vinLen;
            uint8_t vinArray[32];
            uint8_t carVehCfgType;
            uint32_t SID;
            VersionInfo softwareVersion;
            CarConfig carConfig[128];
        };

        // Vehicle data version 3
        struct FT_VehicleDataV3 {
            uint64_t TimeStamp;
            uint8_t SpeedValidity;
                uint8_t	SpeedValiditySDF;
                float32_t	Speed;
                uint8_t	YawRateValidity;
                float32_t	YawRate;
                uint8_t	VehicleMotionStatusValidity;
                uint8_t	VehicleMotionStatus;
                uint8_t	accelerationValidity;
                float32_t	acceleration;
                uint8_t	lateralAccelerationValidity;
                float32_t	lateralAcceleration;
                uint8_t	GearSwitchPositionValidity;
                uint8_t	GearSwitchPosition;
                uint8_t	HighBeamStaValidity;
                uint8_t	HighBeamSta;
                uint8_t	LowBeamStaValidity;
                uint8_t	LowBeamSta;
                uint8_t	fogBeamFrontStaValidity;
                uint8_t	FogBeamFrontSta;
                uint8_t	fogBeamRearStaValidity;
                uint8_t	FogBeamRearSta;
                uint8_t	TurnSignRightStaValidity;
                uint8_t	TurnSignRightSta;
                uint8_t	TurnSignLeftStaValidity;
                uint8_t	TurnSignLeftSta;
                uint8_t	WiperStaValidity;
                uint8_t	WiperSta;
                uint8_t	BonnetStaValidity;
                uint8_t	BonnetSta;
                uint8_t	TrunkStaValidity;
                uint8_t	TrunkSta;
                uint8_t	FrontLeftDoorStaValidity;
                uint8_t	FrontLeftDoorSta;
                uint8_t	mirrFoldStsAtDrvr;
                uint8_t	FrontRightDoorStaValidity;
                uint8_t	FrontRightDoorSta;
                uint8_t	mirrFoldStsAtPass;
                uint8_t	RearLeftDoorStaValidity;
                uint8_t	RearLeftDoorSta;
                uint8_t	RearRightDoorStaValidity;
                uint8_t	RearRightDoorSta;
                uint8_t	isSuspPosnVertLe1FrntValidity;
                float32_t	isSuspPosnVertLe1Frnt;
                uint8_t	isSuspPosnVertLe1ReValidity;
                float32_t	isSuspPosnVertLe1Re;
                uint8_t	isSuspPosnVertRi1SuspPosnVertRiFrntValidity;
                float32_t	isSuspPosnVertRi1SuspPosnVertRiFrnt;
                uint8_t	isSuspPosnVertRi1SuspPosnVertRiReValidity;
                float32_t	isSuspPosnVertRi1SuspPosnVertRiRe;
                uint8_t	PinionSteerAgValidity;
                float32_t	PinionSteerAg;
                uint8_t	PinionSteerAgSpdValidity;
                float32_t	PinionSteerAgSpd;
                uint8_t	acceleratorPedalPosnValidity;
                float32_t	acceleratorPedalPosn;
                uint8_t	mstCylBrkPressureValidity;
                float32_t	mstCylBrkPressure;
                uint8_t	brkPedlStatusStaValidity;
                uint8_t	brkPedlStatusSta;
                uint8_t	IndcrTurnLightValidity;
                uint8_t	IndcrTurnLight;
                uint8_t	steeringWheelAngleValidity;
                float32_t	steeringWheelAngle;
                uint8_t	steeringWheelSpeedValidity;
                float32_t	steeringWheelSpeed;
                uint8_t	wheelSpeedFrontLeftValidity;
                float32_t	wheelSpeedFrontLeft;
                uint8_t	wheelSpeedFrontRightValidity;
                float32_t	wheelSpeedFrontRight;
                uint8_t	wheelSpeedRearLeftValidity;
                float32_t	wheelSpeedRearLeft;
                uint8_t	wheelSpeedRearRightValidity;
                float32_t	wheelSpeedRearRight;
                uint8_t	brakePedalPositionValidity;
                float32_t	brakePedalPosition;
                uint8_t	warningLightStaValidity;
                uint8_t	warningLightSta;
                uint8_t	stopLightStaValidity;
                uint8_t	stopLightSta;
                uint8_t	sideLightStaValidity;
                uint8_t	sideLightSta;
                uint8_t    vinCodeValidity;
                uint8_t    vinCode;
                uint8_t	pluseFrontLeftValidity;
                uint8_t	pluseFrontLeft;
                uint8_t	pluseFrontRightValidity;
                uint8_t	pluseFrontRight;
                uint8_t	pluseRearLeftValidity;
                uint8_t	pluseRearLeft;
                uint8_t	pluseRearRightValidity;
                uint8_t	pluseRearRight;
                uint8_t	whlDirRotlFrntLeValidity;
                uint8_t	whlDirRotlFrntLe;
                uint8_t	whlDirRotlFrntRiValidity;
                uint8_t	whlDirRotlFrntRi;
                uint8_t	whlDirRotlReLeValidity;
                uint8_t	whlDirRotlReLe;
                uint8_t	whlDirRotlReRiValidity;
                uint8_t	whlDirRotlReRi;
                uint8_t	whlAngularSpdFrntLeValidity;
                float32_t	whlAngularSpdFrntLe;
                uint8_t	whlAngularSpdFrntRiValidity;
                float32_t	whlAngularSpdFrntRi;
                uint8_t	whlAngularSpdReLeValidity;
                float32_t	whlAngularSpdReLe;
                uint8_t	whlAngularSpdReRiValidity;
                float32_t	whlAngularSpdReRi;
                uint8_t	vehicleDataUpdateflag;
                uint8_t	driveMode;
                uint8_t	handBrake;
                uint8_t	longitudinalControllerStatus;
                uint8_t	lateralControllerStatusValidity;
                uint8_t	lateralControllerStatus;
                uint8_t	drivepedalStatusValidity;
                uint8_t	drivepedalStatus;
                uint8_t	RoadInclnValidity;
                float32_t	RoadIncln;
                uint8_t	vehicleType;
                float32_t	DrvrDecelReqFloat8;
                uint8_t	CrsCtrlOvrdn;
                uint8_t	SuspHeiLvlIndcnValidity;
                uint8_t	SuspHeiLvlIndcn;
                uint8_t	LeFrntTireMsgPValidity;
                float32_t	LeFrntTireMsgP;
                uint8_t	LeReTireMsgPValidity;
                float32_t	LeReTireMsgP;
                uint8_t	RiFrntTireMsgPValidity;
                float32_t	RiFrntTireMsgP;
                uint8_t	RiReTireMsgPValidity;
                float32_t	RiReTireMsgP;
                uint8_t	WhlLockStatus;
                uint8_t	DrvgDirDesValidity;
                uint8_t	DrvgDirDes;
                uint8_t	AutPrkgHandSlotNrReqValidity;
                uint8_t	AutPrkgHandSlotNrReq;
                uint8_t	AutPrkgSlotNrReqValidity;
                uint8_t	AutPrkgSlotNrReq;
                uint8_t	LampReqByVehHldValidity;
                uint8_t	LampReqByVehHld;
                uint8_t SapaVehMtnstVehMtnst;
        };

        // Emergency stop information
        struct EStop {
            bool eStop;
            uint8_t eStopReason;
        };

        // Trajectory point with motion information
        struct TrajectoryPoint {
            Point3F point;
            float32_t heading;
            int16_t t;
            float32_t kappa;
            float32_t dkappa;
            float32_t velocity;
            float32_t acceleration;
            float32_t jerk;
            int16_t s;
        };

        // Park planning data output
        struct ParkPlanningData {
            DataHeader header;
            Point3F targetPoint;
            float32_t targetHeading;
            float32_t trajectoryLength;
            float32_t nextTrajectoryLength;
            float32_t trajectoryPeriod;
            uint8_t gear;
            EStop eStop;
            uint8_t decisionResult;
            uint8_t turnLight;
            uint8_t beam;
            bool isHold;
            uint8_t ScenairoType;
            uint8_t planningType;
            TrajectoryPoint trajectoryPoints[60];
            uint16_t trajectoryPointsValidSize;
            float32_t totalTime;
            uint8_t reserve[8];
            float32_t accRequest;
            float32_t accUpperLimit;
            float32_t accLowerLimit;
            float32_t jerkUpperLimit;
            float32_t jerkLowerLimit;
            uint8_t epbRequest;
            uint8_t planningStatus;
            float32_t distanceToCIPV;
            uint8_t ACCMode;
            float32_t VehicleRotationAngle;
            uint8_t WheelLockChoice;
            uint8_t MirrOpenClsReq;
            float32_t steerRequest;
            uint8_t isSapa;
        };

        // Algorithm initialization status
        struct AlgInitSts {
            TimeStamp stamp;
            uint8_t Alg_Index;
            uint8_t InitStatus;
            uint8_t taskflowID;
            uint8_t rev[8];
        };

        // Park planning state
        struct ParkPlaningState {
            DataHeader header;
            uint8_t smBackGrndSlotAvail;
            uint8_t    smPathPlanning;
            uint8_t    smParkComplete;
            uint8_t    smPrkgProgs;
            bool    smWheelStuck;
            bool    pathOutPlanning;
            uint8_t    parkSlotType;
            uint8_t    smLastParkSlotType;
            uint8_t    smCollisionRiskType;
            float32_t    smRspaMoveDistance;
            uint8_t    smRpaLeftParkOut;
            uint8_t    smRpaRightParkOut;
            uint16_t    DstToCllsnOfPrkgEmgyBrk;
            uint8_t    EmgyBrkActiveReq;
            uint8_t    DPAMode;
            uint8_t    FuncEnabledConfirm;
            uint8_t    rev[4];
            uint8_t    ParkInOffsetDir;
            uint8_t    controlMode;
            uint8_t    FrntAndRePrkgInSts;
            uint8_t    CustSlotSts;
            uint8_t RevTemp[10];
        };

        // Planning slot information
        struct PlanningSlotInfo {
            uint32_t preSelectSlotId;
            bool isNarrowSlot;
            uint8_t recommendLevel;
        };

        // RPA slot information
        struct RPASlotInfo {
            uint16_t PointACoorn[2];
            uint16_t PointBCoorn[2];
            uint8_t PointCLocn;
            uint8_t SlotType;
        };

        // Park planning information
        struct ParkPlanningInfo {
            DataHeader header;
            PlanningSlotInfo planningSlotInfo[20];
            uint8_t    prkgProgs;
            float32_t    parkRemainDistance;
            uint8_t    planningSlotInfoSize;
            bool    isVehicleCornerPointValid;
            Point3F    vehicleCornerPoint[4];
            Point3F    targetSlotCornerPoint[4];
            uint8_t    parkOutDirection;
            uint8_t    cruisingRecPed;
            uint8_t    cruisingRecVeh;
            uint32_t    planningConfirmSlotId;
            RPASlotInfo    rpaSlotInfo;
            uint8_t    ParkInOffsetDir;
            uint8_t    CustSlotSts;
            int16_t    CustPrkSlotPosnAngleDisp;
            int16_t    CustPrkSlotPosnEndX1;
            int16_t    CustPrkSlotPosnEndX2;
            int16_t    CustPrkSlotPosnEndY1;
            int16_t    CustPrkSlotPosnEndY2;
            int16_t    CustPrkSlotPosnStartX1;
            int16_t    CustPrkSlotPosnStartX2;
            int16_t    CustPrkSlotPosnStartY1;
            int16_t    CustPrkSlotPosnStartY2;
            uint8_t    CustPrkOpenEdge;
            int16_t    PrkgCaliSlotPosnAngleDisp;
            int16_t    PrkgCaliSlotPosnEndX1;
            int16_t    PrkgCaliSlotPosnEndX2;
            int16_t    PrkgCaliSlotPosnEndY1;
            int16_t    PrkgCaliSlotPosnEndY2;
            int16_t    PrkgCaliSlotPosnStartX1;
            int16_t    PrkgCaliSlotPosnStartX2;
            int16_t    PrkgCaliSlotPosnStartY1;
            int16_t    PrkgCaliSlotPosnStartY2;
            uint8_t rev[6];
        };

        // Park path point
        struct ParkPathPoint {
            Point3F point;
            float32_t heading;
        };

        // Park planning UI information
        struct ParkPlanningUIInfo {
            ParkPathPoint parkPathPoint[128];
            uint8_t rev[8];
        };

        // Park planning debug information
        struct ParkPlanningDebug {
            uint8_t decisionType;
            TrajectoryPoint fullTrajectory[150];
            TrajectoryPoint winnerFsPath[30];
            TrajectoryPoint leftOffsetLine[30];
            TrajectoryPoint rightOffsetLine[30];
            uint8_t flowtype;
            uint8_t errorCode;
            Point3F targetPoint[10];
            uint8_t detourState;
            uint8_t cutInType;
            uint8_t cutOutType;
            uint8_t blockType;
        };
        struct Planning_TrajectoryPoint
        {
            TrajectoryPoint Trajectory;
            uint8_t GearPosition; 
        };
    }

}

#endif //J6B_PARKING_PNC_APS_PLANNING_DATATYPE_H