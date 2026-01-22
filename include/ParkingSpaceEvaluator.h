//
// Created by lxj on 2026/1/19.
//

#ifndef J6B_PARKING_PNC_PARKINGSPACEEVALUATOR_H
#define J6B_PARKING_PNC_PARKINGSPACEEVALUATOR_H
#include "Config_Reader.h"
#include "APS_Planning_Datatype.h"
using J6B_AD::APS_Planning::APAFusionOutput;
using J6B_AD::APS_Planning::SlotFusionResult;

namespace APS_ParkingSpace {
    class ParkingSpaceEvaluator {
        public:
            void ParkingSpaceEvaluate(const APAFusionOutput& APAFusionOutput);
              std::vector<SlotFusionResult> GetValidParkingSlot();
        private:
            std::vector<SlotFusionResult> ValidParkingSlot;
    };
} // APS_ParkingSpace

#endif //J6B_PARKING_PNC_PARKINGSPACEEVALUATOR_H