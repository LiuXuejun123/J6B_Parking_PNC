//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_PARKINGSPACERECOMMENDER_H
#define J6B_PARKING_PNC_PARKINGSPACERECOMMENDER_H
#include "Config_Reader.h"
#include "APS_Planning_Datatype.h"
#include "Planning_Math.h"
using J6B_AD::APS_Planning::Point3F;
using J6B_AD::APS_Planning::PlanningSlotInfo;
using J6B_AD::APS_Planning::SlotFusionResult;
using J6B_AD::APS_Planning::LocationData;
namespace APS_ParkingSpace {
    struct ParkingslotRec
    {
        uint32_t id;
        double distance;
    };
    class ParkingSpaceRecommender {
        public:
            void ParkingSpaceRecommendProcess(const std::vector<SlotFusionResult>& ValidSlot,
                                              const LocationData& LocationData,
                                              u_int32_t selectedid,
                                              uint8_t apaSubSysSts);
            std::vector<PlanningSlotInfo> Get_ValidPalnningSlotList();

        private:
            std::vector<PlanningSlotInfo> ValidPalnningSlotList_;
            std::vector<APS_ParkingSpace::ParkingslotRec> slotlist_;

    };
} // APS_ParkingSpace

#endif //J6B_PARKING_PNC_PARKINGSPACERECOMMENDER_H