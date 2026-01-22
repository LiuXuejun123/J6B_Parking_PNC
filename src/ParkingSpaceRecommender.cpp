//
// Created by lxj on 2026/1/19.
//

#include "ParkingSpaceRecommender.h"

namespace APS_ParkingSpace {
    void ParkingSpaceRecommender::ParkingSpaceRecommendProcess(const std::vector<J6B_AD::APS_Planning::SlotFusionResult>& ValidSlot,
                                                                const J6B_AD::APS_Planning::LocationData& LocationData,
                                                                u_int32_t selectedid,
                                                                uint8_t apaSubSysSts)
    {
        if (ValidSlot.empty())
        {
            return;
        }

        if (selectedid != u_int32_t(0)&&(apaSubSysSts == uint8_t(9) || apaSubSysSts == uint8_t(4)))
        {
            this->ValidPalnningSlotList_.clear();
            for(size_t i=0;i<ValidSlot.size();i++)
            {
                if(ValidSlot[i].id == selectedid)
                {
                   PlanningSlotInfo temp;
                   temp.isNarrowSlot = false;
                   temp.preSelectSlotId = selectedid;
                   temp.recommendLevel = u_int8_t(1);
                   this->ValidPalnningSlotList_.emplace_back(temp);
                   return;
                }
            }
        }  
        this->slotlist_.clear();
        this->ValidPalnningSlotList_.clear();// TIPS ！！！！
        for(size_t i=0;i<ValidSlot.size();i++)
        {
            Point3F Center = utils::GetQuadrilateralCenter(ValidSlot[i].entrancePointA,ValidSlot[i].entrancePointB,
                                                            ValidSlot[i].tailPointC,ValidSlot[i].tailPointD);
            Point3F LocationPos;
            LocationPos.x = LocationData.pose.position.x;
            LocationPos.y = LocationData.pose.position.y;
            LocationPos.z = LocationData.pose.position.z;
                 
            ParkingslotRec temp;
            temp.id = ValidSlot[i].id;
            temp.distance = utils::DistanceSquared(Center,LocationPos);
            this->slotlist_.push_back(temp);
        }
        std::sort(this->slotlist_.begin(), this->slotlist_.end(),
                    [](const ParkingslotRec& a, const ParkingslotRec& b) {
                    return a.distance < b.distance;           // 升序
        // 或 return a.dist_sq > b.dist_sq;     // 降序
        // 或多条件：先比距离，再比 id
        // return a.dist_sq != b.dist_sq ? a.dist_sq < b.dist_sq : a.id < b.id;
                    });

        u_int8_t recommendlevel = u_int8_t(1);
        for (size_t i =0; i < this->slotlist_.size();i++)
        {
            PlanningSlotInfo temp;
            temp.isNarrowSlot = false;
            temp.preSelectSlotId = this->slotlist_[i].id;
            temp.recommendLevel = recommendlevel;
            this->ValidPalnningSlotList_.emplace_back(temp);
            recommendlevel ++;
        }
        
    }

      std::vector<PlanningSlotInfo> ParkingSpaceRecommender::Get_ValidPalnningSlotList()
    {

        return this->ValidPalnningSlotList_;
    }

} // APS_ParkingSpace