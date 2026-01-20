//
// Created by lxj on 2026/1/19.
//

#include "ParkingSpaceEvaluator.h"

namespace APS_ParkingSpace {

    void ParkingSpaceEvaluator::ParkingSpaceEvaluate(const J6B_AD::APS_Planning::APAFusionOutput &APAFusionOutput)
    {
        this->ValidParkingSlot.clear();
        for(size_t i = 0; i<APAFusionOutput.pld.pldFusionResultVaildSize;i++)
        {
           if (APAFusionOutput.pld.pldFusionResult[i].OccType == u_int32_t(0))
           {
                J6B_AD::APS_Planning::SlotFusionResult Tempslot;
                Tempslot = APAFusionOutput.pld.pldFusionResult[i];
                this->ValidParkingSlot.emplace_back(Tempslot);
           }
        }
    }

    inline std::vector<J6B_AD::APS_Planning::SlotFusionResult> ParkingSpaceEvaluator::GetValidParkingSlot()
    {
        return this->ValidParkingSlot;
    }

} // APS_ParkingSpace