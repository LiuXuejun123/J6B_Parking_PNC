//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_PARKINGSPACEEVALUATOR_H
#define J6B_PARKING_PNC_PARKINGSPACEEVALUATOR_H
#include "common/config_reader/Config_Reader.h"
#include "common/datatype/APS_Planning_Datatype.h"
namespace APS_ParkingSpace {
    class ParkingSpaceEvaluator {
        public:
            void ParkingSpaceEvaluate(const J6B_AD::APS_Planning::APAFusionOutput& APAFusionOutput);
              std::vector<J6B_AD::APS_Planning::SlotFusionResult> GetValidParkingSlot();
        private:
            std::vector<J6B_AD::APS_Planning::SlotFusionResult> ValidParkingSlot;
    };
} // APS_ParkingSpace

#endif //J6B_PARKING_PNC_PARKINGSPACEEVALUATOR_H