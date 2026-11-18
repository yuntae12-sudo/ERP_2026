#include <Planning/Planning.hpp>
extern bool b_SlowSignal, b_StaySignal;

using namespace std;

void MakeRefPath(Planning_t &st_Planning, GPSIMU_t &st_GPSIMU, LiDAR_t &st_LiDAR)
{
	st_Planning.st_RefPath.s32_PathNum = st_Planning.s32_RefPathEndIdx;
    float32_t NearDist = Normalize2D(st_GPSIMU.f32_E, st_GPSIMU.f32_N, st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx], st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx]);
    Point2D_t PathVector = UnitVector(st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx], st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx], 
                                st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx + 1], st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx + 1]);
    Point2D_t ERPVector = UnitVector(st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx], st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx], 
                                st_GPSIMU.f32_E, st_GPSIMU.f32_N);
    bool ERPCrossProduct = CrossProduct(PathVector, ERPVector);

    // cout << "NearDist: " << NearDist << endl;
    // cout << "ERPCrossProduct: " << ERPCrossProduct << endl;
    // cout << "Slope: " << st_LiDAR.f32_Slope << endl;
    float32_t f32_RefYaw = 0;
    float32_t f32_MyYaw = st_GPSIMU.f32_Heading_deg;
    float32_t f32_LaneOffSet = 0;

    for(int32_t s32_i = 0; s32_i < 4; s32_i++)
    {
        if(st_Planning.s32_GlobalNearIdx > st_Planning.ars32_MakePathStart[s32_i] && st_Planning.s32_GlobalNearIdx < st_Planning.ars32_MakePathEnd[s32_i])
        {
            st_Planning.b_RefPathIdx = true;
            f32_RefYaw = st_Planning.arf32_RefYaw[s32_i];
            f32_LaneOffSet = st_Planning.arf32_LaneOffSet[s32_i];
            break;
        }
        else
        {
            st_Planning.b_RefPathIdx = false;
        }
    }

    float32_t f32_LaneSlope = st_LiDAR.f32_Slope;
    // float32_t f32_LaneAngle_rad = Deg2Rad(st_LiDAR.f32_Angle_deg);
    st_LiDAR.f32_Angle_deg = f32_MyYaw - f32_RefYaw;
    float32_t f32_LaneAngle_rad = Deg2Rad(st_LiDAR.f32_Angle_deg);
    // cout << "st_LiDAR.s32_LaneSize: " << st_LiDAR.s32_LaneSize << endl;
    // cout << "st_LiDAR.f32_Angle_deg: " << st_LiDAR.f32_Angle_deg << endl;
    // cout << "f32_LaneAngle_rad: " << f32_LaneAngle_rad << endl;
	// if(st_Planning.s32_GlobalNearIdx > 1200 && NearDist < 0.2 && st_Planning.b_MakeRefPathFirst == true)
    // Katri : 1200
    // if(st_LiDAR.s32_LaneSize != 0 && st_Planning.s32_GlobalNearIdx > 1500 && st_Planning.b_MakeRefPathFirst == true)
    if(st_Planning.b_RefPathIdx == true && st_Planning.b_MakeRefPathFirst == true)
	{   

        // for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
        // {
        //     st_Planning.st_RefPath.arf32_X[s32_i] = s32_i * 0.1;
        //     st_Planning.st_RefPath.arf32_Y[s32_i] = 0;
        // }

        for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
        {
            st_Planning.st_RefPath.arf32_X[s32_i] = st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx + s32_i] - st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx];
            st_Planning.st_RefPath.arf32_Y[s32_i] = st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx + s32_i] - st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx];
        }
        Point2D_t RefPathVector = UnitVector(st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx], st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx], 
                                    st_Planning.st_GlobalPath.arf32_E[st_Planning.s32_GlobalNearIdx + 1], st_Planning.st_GlobalPath.arf32_N[st_Planning.s32_GlobalNearIdx + 1]);
        float32_t f32_RefPath_rad = atan2(RefPathVector.f32_Y, RefPathVector.f32_X);

        for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
        {
            float32_t tmp_X = st_Planning.st_RefPath.arf32_X[s32_i];
            float32_t tmp_Y = st_Planning.st_RefPath.arf32_Y[s32_i];

            st_Planning.st_RefPath.arf32_X[s32_i] = (cos(-f32_RefPath_rad) * tmp_X) 
                                                    - (sin(-f32_RefPath_rad) * tmp_Y);
            st_Planning.st_RefPath.arf32_Y[s32_i] = (sin(-f32_RefPath_rad) * tmp_X) 
                                                    + (cos(-f32_RefPath_rad) * tmp_Y);
        }

        if(ERPCrossProduct > 0)
        {
            for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
            {
                float32_t tmp_X = st_Planning.st_RefPath.arf32_X[s32_i];
                float32_t tmp_Y = st_Planning.st_RefPath.arf32_Y[s32_i];

                st_Planning.st_RefPath.arf32_X[s32_i] = (cos(f32_LaneAngle_rad) * tmp_X) 
                                                        - (sin(f32_LaneAngle_rad) * tmp_Y);
                st_Planning.st_RefPath.arf32_Y[s32_i] = (sin(f32_LaneAngle_rad) * tmp_X) 
                                                        + (cos(f32_LaneAngle_rad) * tmp_Y)
                                                        - NearDist + f32_LaneOffSet;

            }
        }
        else
        {
            for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
            {
                float32_t tmp_X = st_Planning.st_RefPath.arf32_X[s32_i];
                float32_t tmp_Y = st_Planning.st_RefPath.arf32_Y[s32_i];
                
                st_Planning.st_RefPath.arf32_X[s32_i] = (cos(f32_LaneAngle_rad) * tmp_X) 
                                                        - (sin(f32_LaneAngle_rad) * tmp_Y);
                st_Planning.st_RefPath.arf32_Y[s32_i] = (sin(f32_LaneAngle_rad) * tmp_X) 
                                                        + (cos(f32_LaneAngle_rad) * tmp_Y)
                                                        + NearDist + f32_LaneOffSet;
            }
        }

		st_GPSIMU.f32_PreE = st_GPSIMU.f32_E;
		st_GPSIMU.f32_PreN = st_GPSIMU.f32_N;
		st_GPSIMU.f32_PreHeading_rad = st_GPSIMU.f32_Heading_rad;
		st_GPSIMU.f32_RefHeading_rad = 0;

		st_Planning.b_MakeRefPathFirst = false;
	}

	float32_t DeltaDist = Normalize2D(st_GPSIMU.f32_PreE, st_GPSIMU.f32_PreN, st_GPSIMU.f32_E, st_GPSIMU.f32_N);
    float32_t DeltaYaw = st_GPSIMU.f32_Heading_rad - st_GPSIMU.f32_PreHeading_rad;
    if(DeltaYaw > 0.2)
    {
        DeltaYaw = 0;
    }
	// float32_t DeltaYaw = 0;
    st_GPSIMU.f32_RefHeading_rad += DeltaYaw;


    float32_t f32_X = 0;
    float32_t f32_Y = 0;

    for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
    {
        float32_t tmp_X = st_Planning.st_RefPath.arf32_X[s32_i];
        float32_t tmp_Y = st_Planning.st_RefPath.arf32_Y[s32_i];

        st_Planning.st_RefPath.arf32_X[s32_i] = (cos(DeltaYaw) * tmp_X ) 
                                                - (sin(DeltaYaw) * tmp_Y) - (cos(DeltaYaw) * DeltaDist - sin(DeltaYaw) * 0);
        st_Planning.st_RefPath.arf32_Y[s32_i] = (sin(DeltaYaw) * tmp_X ) 
                                                + (cos(DeltaYaw) * tmp_Y) - (sin(DeltaYaw) * DeltaDist + cos(DeltaYaw) * 0);
    }
    st_Planning.st_RefPath.s32_NearIdx = FindNear_LiDAR(st_Planning, f32_X, f32_Y, st_Planning.st_RefPath.s32_PathNum);
    // cout << "st_Planning.st_RefPath.s32_NearIdx: " << st_Planning.st_RefPath.s32_NearIdx << endl;
    // for(int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i++)
    // {
    //     cout << "(" << st_Planning.st_RefPath.arf32_X[s32_i] << ", " << st_Planning.st_RefPath.arf32_Y[s32_i] << ")" << endl;
    // }



	st_GPSIMU.f32_PreE = st_GPSIMU.f32_E;
	st_GPSIMU.f32_PreN = st_GPSIMU.f32_N;
	st_GPSIMU.f32_PreHeading_rad = st_GPSIMU.f32_Heading_rad;
	
    // cout << "st_GPSIMU.f32_RefHeading_rad: " << st_GPSIMU.f32_RefHeading_rad << endl;

    for(int32_t s32_i = 0; s32_i < 4; s32_i++)
    {
        if(st_Planning.s32_GlobalNearIdx < st_Planning.ars32_MakePathStart[s32_i + 1] && st_Planning.s32_GlobalNearIdx > st_Planning.ars32_MakePathEnd[s32_i])
        {
            st_Planning.b_MakeRefPathFirst = true;
        }
    }

}

void FindObjectNear_LiDAR(LiDAR_t &st_LiDAR, Planning_t &st_Planning)
{
	int32_t s32_NearLen = st_Planning.st_RefPath.s32_PathNum;

	for(int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
	{
		int32_t s32_ObjectIdx = FindNear_LiDAR(st_Planning, st_LiDAR.st_XY[s32_i], st_Planning.st_RefPath.s32_PathNum);
        if (s32_i > 0 && abs(st_LiDAR.ars32_ObjectNearIdx[s32_i - 1] - s32_ObjectIdx) < 10)
        {
            st_LiDAR.ars32_ObjectNearIdx[s32_i] = st_LiDAR.ars32_ObjectNearIdx[s32_i - 1] + 10;
        }
        else
        {
            st_LiDAR.ars32_ObjectNearIdx[s32_i] = s32_ObjectIdx;
        }
	}

}

void Perpendicular_LiDAR(Planning_t &st_Planning, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                   int32_t &s32_Iter, float32_t &f32_Gap,
                   std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints)
{
    Path_t st_Path;
    Point2D_t st_UnitVector = {0}, st_LeftDirection = {0}, st_RightDirection = {0};
    std::vector<float32_t> vf_ObjectNearPoint(2, 0);
	
	st_Path = st_Planning.st_RefPath;
    
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        // Tangent Vector get
        if (ars32_ObjectNearIdx[s32_i] < st_Path.s32_PathNum - 1)
        {
            st_UnitVector = UnitVector(st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]],
                                       st_Path.arf32_X[ars32_ObjectNearIdx[s32_i] + 1], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i] + 1]);
        }
        else
        {
            st_UnitVector = UnitVector(st_Path.arf32_X[ars32_ObjectNearIdx[s32_i] - 1], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i] - 1],
                                       st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]);
        }
        // +-Normal Vector get
        st_LeftDirection = RotatePoint(st_UnitVector, -90);
        st_RightDirection = RotatePoint(st_UnitVector, 90);

        // Cal Perpendicular Points
        vf_PerpendicularPoints[s32_i].push_back({st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]],
                                                 st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]}); // ok
        vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin());
        vf_ObjectNearPoint[0] = st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]];
        vf_ObjectNearPoint[1] = st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]];
        
        GetGapPoints(st_Planning, vf_ObjectNearPoint, st_LeftDirection, s32_Iter, f32_Gap, vf_PerpendicularPoints[s32_i]);
        GetGapPoints(st_Planning, vf_ObjectNearPoint, st_RightDirection, s32_Iter, f32_Gap, vf_PerpendicularPoints[s32_i]);
        // std::cout << "Perpendicular -------------------" << std::endl;
        // for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j ++)
        // {
        //     cout << "(" << vf_PerpendicularPoints[s32_i][s32_j][0] << ", " << vf_PerpendicularPoints[s32_i][s32_j][1] << ")" << endl;
        // }
    }
}

void Constraint_LiDAR(Planning_t &st_Planning, Point2D_t *st_XY,  int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, 
                        std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                         float32_t f32_Thres, float32_t f32_LeftThres, float32_t f32_RightThres)
{
	Path_t st_Path;
	st_Path = st_Planning.st_RefPath;
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        for (int32_t s32_j = vf_PerpendicularPoints[s32_i].size() - 1; s32_j >= 0; s32_j--)
        {

            Point2D_t PathVector = UnitVector(st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]], 
                                        st_Path.arf32_X[ars32_ObjectNearIdx[s32_i] + 1], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i] + 1]);
            Point2D_t PerpendiVector = UnitVector(st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]], 
                                        vf_PerpendicularPoints[s32_i][s32_j][0], vf_PerpendicularPoints[s32_i][s32_j][1]);
            bool PerpendiCrossProduct = CrossProduct(PathVector, PerpendiVector);
            // cout << "f32_LeftThres: " << f32_LeftThres << ", f32_RightThres: " << f32_RightThres << endl;
            // if (Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]) > 1.9 ||
            //     Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_XY[s32_i]) < f32_Thres)
            if ((PerpendiCrossProduct == 1 && Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]) > f32_LeftThres) ||
                (PerpendiCrossProduct == 0 && Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]) > f32_RightThres) || 
                Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_XY[s32_i]) < f32_Thres)
            {
                vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            }
            // if (Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]) > 1.7))
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
            // if (Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_XY[s32_i]) < f32_Thres)
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
        }
        // std::cout << "Constraint -------------------" << std::endl;
        // for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j++)
        // {
        //     cout << "(" << vf_PerpendicularPoints[s32_i][s32_j][0] << ", " << vf_PerpendicularPoints[s32_i][s32_j][1] << ")" << endl;
        // }
    }
}

void CostFunction_LiDAR(GPSIMU_t &st_GPSIMU, Planning_t &st_Planning,
                  int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, Point2D_t *st_XY,
                  std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                  Point2D_t *st_OptimalPoints, bool &b_ObtimalExist,
                  float32_t *arf_Costs)
{
	Path_t st_Path;
	st_Path = st_Planning.st_RefPath;
    float32_t f32_X = 0, f32_Y = 0;

    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        if (vf_PerpendicularPoints[s32_i].size() == 1)
        {
            // Find One Optimal Point
            st_OptimalPoints[s32_i] = {vf_PerpendicularPoints[s32_i][0][0],
                                       vf_PerpendicularPoints[s32_i][0][1]};
            continue;
        }
        else if (vf_PerpendicularPoints[s32_i].size() == 0)
        {
            // Not Optimal Point ~= Not Avoid
            b_ObtimalExist = false;
            st_OptimalPoints[s32_i] = {st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]],
                                       st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]};
            return;
        }
        // Calculation Cost - Perpendicular Points
        float32_t arf_Object_PerpendPointsDist[vf_PerpendicularPoints[s32_i].size()] = {0};
        float32_t arf_Path_PerpendPointsDist[vf_PerpendicularPoints[s32_i].size()] = {0};
        float32_t arf_Vehicle_PerpendPointsDist[vf_PerpendicularPoints[s32_i].size()] = {0};
        float32_t arf_OptimalCost[vf_PerpendicularPoints[s32_i].size()] = {0};

        for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j++)
        {
            arf_Object_PerpendPointsDist[s32_j] = Normalize2D(vf_PerpendicularPoints[s32_i][s32_j],
                                                              st_XY[s32_i]);
            // cout << "-------------------------------Object----------------------" << endl;
            // cout << "st_XY[0]: " << st_XY[s32_i].f32_X << ", " << "st_XY[1]: " << st_XY[s32_i].f32_Y << endl;
            // 장애물이랑 포인트랑에 거리 --> 포인트랑 장애물이랑 멀수록 크기가 커져 --> optimal을 뽑을 때는 최소인 애를 구하는거야 --> 그래서 cost가 -??

            arf_Path_PerpendPointsDist[s32_j] = Normalize2D(vf_PerpendicularPoints[s32_i][s32_j],
                                                            st_Path.arf32_X[ars32_ObjectNearIdx[s32_i]],
                                                            st_Path.arf32_Y[ars32_ObjectNearIdx[s32_i]]);
            // 포인트랑 경로 상의 장애물 니어랑 거리 --> 니어점을 기준으로 수직점을 찍어준 포인트랑 니어점이랑 거리 --> 거리랑 회피 경로 간의 거리 --> global 경로랑 안 떨어지겠다 --> cost +

            arf_Vehicle_PerpendPointsDist[s32_j] = Normalize2D(vf_PerpendicularPoints[s32_i][s32_j],
                                                               f32_X, f32_Y);
            // 포인트랑 차량의 거리 --> 고민 좀 해봐

            // arf_OptimalCost[s32_j] = arf_Costs[0] * arf_Object_PerpendPointsDist[s32_j] +
            //                          arf_Costs[1] * arf_Path_PerpendPointsDist[s32_j] +
            //                          arf_Costs[2] * arf_Vehicle_PerpendPointsDist[s32_j];
        }
        // score화 한 거
        float32_t f32_Object_PerpendPointsDist = *std::max_element(arf_Object_PerpendPointsDist, arf_Object_PerpendPointsDist + vf_PerpendicularPoints[s32_i].size());
        float32_t f32_Path_PerpendPointsDist = *std::max_element(arf_Path_PerpendPointsDist, arf_Path_PerpendPointsDist + vf_PerpendicularPoints[s32_i].size());
        float32_t f32_Vehicle_PerpendPointsDist = *std::max_element(arf_Vehicle_PerpendPointsDist, arf_Vehicle_PerpendPointsDist + vf_PerpendicularPoints[s32_i].size());

        for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j++)
        {
            arf_OptimalCost[s32_j] = arf_Costs[0] * arf_Object_PerpendPointsDist[s32_j] / f32_Object_PerpendPointsDist +
                                     arf_Costs[1] * arf_Path_PerpendPointsDist[s32_j] / f32_Path_PerpendPointsDist +
                                     arf_Costs[2] * arf_Vehicle_PerpendPointsDist[s32_j] / f32_Vehicle_PerpendPointsDist;
        }
        // std::cout << "f32_Object_PerpendPointsDist : " << f32_Object_PerpendPointsDist << std::endl;
        // std::cout << "f32_Path_PerpendPointsDist : " << f32_Path_PerpendPointsDist << std::endl;
        // std::cout << "f32_Vehicle_PerpendPointsDist : " << f32_Vehicle_PerpendPointsDist << std::endl;
        // Select Optiaml Point to Cost
        int32_t s32_OptimalIdx = std::distance(arf_OptimalCost, std::min_element(arf_OptimalCost, arf_OptimalCost + vf_PerpendicularPoints[s32_i].size()));
        st_OptimalPoints[s32_i] = {vf_PerpendicularPoints[s32_i][s32_OptimalIdx][0], vf_PerpendicularPoints[s32_i][s32_OptimalIdx][1]};
        // std::cout << "Optimal -------------------------------" << std::endl;
        // std::cout << "(" << st_OptimalPoints[s32_i].f32_X << ", " << st_OptimalPoints[s32_i].f32_Y << ")" << std::endl;
    }
}

void SelectFirstLastPoint_LiDAR(Planning_t &st_Planning,
                          int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, float32_t &f32_FirstDist, float32_t &f32_LastDist,
                          std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx)
{

    int32_t s32_NearLen = st_Planning.s32_RefPathEndIdx - st_Planning.s32_RefPathStartIdx;
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        ps_ObjectFirstLastIdx[s32_i].first = st_Planning.s32_RefPathStartIdx;
        ps_ObjectFirstLastIdx[s32_i].second = st_Planning.s32_RefPathEndIdx;
        for (int32_t s32_j = st_Planning.s32_RefPathStartIdx; s32_j < ars32_ObjectNearIdx[s32_i]; s32_j++)
        {
            // Object의 near에서부터 ~m 떨어진 지점
            if (Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y, s32_j, ars32_ObjectNearIdx[s32_i]) < f32_FirstDist)
            {
                ps_ObjectFirstLastIdx[s32_i].first = s32_j;
                break;
            }
        }
        for (int32_t s32_j = ars32_ObjectNearIdx[s32_i]; s32_j < st_Planning.s32_RefPathEndIdx; s32_j++)
        {
            if (Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y, s32_j, ars32_ObjectNearIdx[s32_i]) > f32_LastDist)
            {
                ps_ObjectFirstLastIdx[s32_i].second = s32_j;
                break;
            }
        }
    }
    // std::cout << "Map----------------------------------------" << std::endl;
    // for (int32_t s32_i = st_Planning.s32_RefPathStartIdx; s32_i < st_Planning.s32_RefPathEndIdx; s32_i++)
    // {
    //     std::cout << "(" << st_Planning.st_RefPath.arf32_X[s32_i] << ", " << st_Planning.st_RefPath.arf32_Y[s32_i] << ")" << std::endl;
    // }
    // std::cout << "FirstLast--------------------------------------" << std::endl;
    // for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    // {
        // std::cout << "(" << st_Planning.st_RefPath.arf32_X[ps_ObjectFirstLastIdx[s32_i].first] << ", " << st_Planning.st_RefPath.arf32_Y[ps_ObjectFirstLastIdx[s32_i].first] << ")" << std::endl;
        // std::cout << "(" << st_Planning.st_RefPath.arf32_X[ps_ObjectFirstLastIdx[s32_i].second] << ", " << st_Planning.st_RefPath.arf32_Y[ps_ObjectFirstLastIdx[s32_i].second] << ")" << std::endl;
    // }
}

void ObjectLinked_LiDAR(LiDAR_t &st_LiDAR,
                  std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx, std::vector<std::vector<int32_t>> &vs_LinkedList, int32_t &s32_LinkedObjectNum)
{
    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        // 처음은 무조건 넣고
        // if (s32_LinkedObjectNum == 0)
        if (s32_i == 0)
        {
            // Linked 끄고 싶으면 위 s32_i 를 s32_LinkedObjectNum 으로 변경
            vs_LinkedList[s32_LinkedObjectNum].emplace_back(s32_i);
            continue;
        }
        // 그 이후
        if (ps_ObjectFirstLastIdx[s32_i - 1].second > ps_ObjectFirstLastIdx[s32_i].first)
            vs_LinkedList[s32_LinkedObjectNum].emplace_back(s32_i);
        else
        {
            s32_LinkedObjectNum += 1;
            vs_LinkedList[s32_LinkedObjectNum].emplace_back(s32_i);
        }
    }

    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        vs_LinkedList[s32_i].erase(vs_LinkedList[s32_i].begin());
    }
    if (s32_LinkedObjectNum == 0)
    {
        s32_LinkedObjectNum += 1;
    }
}

void MakeAvoidPath_LiDAR(LiDAR_t &st_LiDAR, Planning_t &st_Planning,
                   Point2D_t *st_OptimalPoints, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx,
                   std::vector<std::vector<int32_t>> &vs_LinkedList, int32_t &s32_LinkedObjectNum)
{
    int32_t s32_AvoidStartIdx = 0, s32_AvoidLastIdx = 0;
    Point2D_t st_AvoidDirection = {0};
    float32_t f32_AvoidDist = 0;
    int32_t s32_StartLocalIdx = st_Planning.st_RefPath.s32_NearIdx;
    st_Planning.st_LocalPath.s32_PathNum = 0;

    // 현재 Near부터 첫번째 물체의 first까지 맵 넣기
    MakePath_LiDAR(st_Planning, st_Planning.st_RefPath.s32_NearIdx, ps_ObjectFirstLastIdx[0].first, st_AvoidDirection, f32_AvoidDist);
    // Linked된 물체에 대해서 값 넣기
    for (int32_t s32_i = 0; s32_i < s32_LinkedObjectNum; s32_i++)
    {
        if (s32_i != 0)
        {
            // Linked된 물체들 중 이전의 맨 마지막과 현재의 맨 처음까지 이어주기
            st_AvoidDirection.f32_X = 0;
            st_AvoidDirection.f32_Y = 0;
            f32_AvoidDist = 0;
            cout << "aaaaaaaaaa" << endl;
            MakePath_LiDAR(st_Planning, ps_ObjectFirstLastIdx[vs_LinkedList[s32_i - 1].back()].second,
                     ps_ObjectFirstLastIdx[vs_LinkedList[s32_i].front()].first, st_AvoidDirection, f32_AvoidDist);
        }
        // Linked된 물체에 대해 시작, 끝 점 선택
        s32_AvoidStartIdx = ps_ObjectFirstLastIdx[vs_LinkedList[s32_i].front()].first;
        s32_AvoidLastIdx = ps_ObjectFirstLastIdx[vs_LinkedList[s32_i].back()].second;
        if (s32_StartLocalIdx > s32_AvoidStartIdx)
        {
            s32_StartLocalIdx = s32_AvoidStartIdx;
        }
        // std::cout << "vs_LinkedList[s32_i].size() : " << vs_LinkedList[s32_i].size()  << std::endl;
        for (int32_t s32_j = 0; s32_j < vs_LinkedList[s32_i].size(); s32_j++)
        {
            st_AvoidDirection = UnitVector(st_Planning.st_RefPath.arf32_X[st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]]],
                                           st_Planning.st_RefPath.arf32_Y[st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]]],
                                           st_OptimalPoints[vs_LinkedList[s32_i][s32_j]].f32_X, st_OptimalPoints[vs_LinkedList[s32_i][s32_j]].f32_Y);

            f32_AvoidDist = Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y, st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]],
                                        st_OptimalPoints[vs_LinkedList[s32_i][s32_j]]);
            // std::cout << "f32_AvoidDist : " << f32_AvoidDist << std::endl;
            // nan 뜨는거 예외처리
            // if (isnan(st_AvoidDirection.f32_X) || isnan(st_AvoidDirection.f32_Y))
            // {
            //     st_AvoidDirection.f32_X = st_Planning.st_PreAvoidDirection[s32_j].f32_X;
            //     st_AvoidDirection.f32_Y = st_Planning.st_PreAvoidDirection[s32_j].f32_Y;
            //     f32_AvoidDist = st_Planning.f32_PreAvoidDist[s32_j];
            // }
            st_Planning.st_PreAvoidDirection[s32_j].f32_X = st_AvoidDirection.f32_X;
            st_Planning.st_PreAvoidDirection[s32_j].f32_Y = st_AvoidDirection.f32_Y;
            st_Planning.f32_PreAvoidDist[s32_j] = f32_AvoidDist;
            if (isnan(st_AvoidDirection.f32_X) || isnan(st_AvoidDirection.f32_Y))
            {
                st_AvoidDirection.f32_X = 0;
                st_AvoidDirection.f32_Y = 0;
            }
            // 첫번째 Linked 물체 중에 1개면
            if (vs_LinkedList[s32_i].size() == 1)
            {
                cout << "bbbbbbbbbbbbb" << endl;
                MakePath_LiDAR(st_Planning, s32_AvoidStartIdx, s32_AvoidLastIdx, st_AvoidDirection, f32_AvoidDist);
                break;
            }
            // Linked된 물체에 대해서
            if (s32_j == 0)
            {
                // 감지된 모든 물체가 Linked됐다고 하고 각 Near 가운데 값까지만 할떄
                s32_AvoidStartIdx = ps_ObjectFirstLastIdx[vs_LinkedList[s32_i][s32_j]].first;
                s32_AvoidLastIdx = (st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]] + st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j + 1]]) / 2;
            }
            else if (s32_j == vs_LinkedList[s32_i].size() - 1)
            {
                s32_AvoidStartIdx = (st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]] + st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j - 1]]) / 2;
                s32_AvoidLastIdx = ps_ObjectFirstLastIdx[vs_LinkedList[s32_i][s32_j]].second;
            }
            else
            {
                s32_AvoidStartIdx = (st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]] + st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j - 1]]) / 2;
                s32_AvoidLastIdx = (st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]] + st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j + 1]]) / 2;
            }
            if (s32_StartLocalIdx > s32_AvoidStartIdx)
            {
                s32_StartLocalIdx = s32_AvoidStartIdx;
            }
            // 계산된 방향과 길이에 대해서 Start부터 Last까지 맵 넣기
            // if (st_Planning.s32_GlobalNearIdx > s32_AvoidStartIdx)
            // {
            //     s32_AvoidStartIdx = st_Planning.s32_GlobalNearIdx;
            // }
            // if(f32_AvoidDist != 0)
            // {
            MakePath_LiDAR(st_Planning, s32_AvoidStartIdx, s32_AvoidLastIdx, st_AvoidDirection, f32_AvoidDist);
            // cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
            // cout << "st_AvoidDirection : " << atan2(st_AvoidDirection.f32_Y, st_AvoidDirection.f32_X) << endl;
            // cout << "f32_AvoidDist : " << f32_AvoidDist << endl;
            // }
            

            // if(s32_i == s32_LinkedObjectNum - 1 && s32_j == vs_LinkedList[s32_i].size() - 1)
            // {
            //     st_AvoidDirection.f32_X = 0;
            //     st_AvoidDirection.f32_Y = 0;
            //     f32_AvoidDist = 0;
            //     MakePath_LiDAR(st_Planning, ps_ObjectFirstLastIdx[st_LiDAR.s32_ObjectNum - 1].second, st_Planning.s32_RefPathEndIdx, st_AvoidDirection, f32_AvoidDist);
            // }
        }
    }
    // 맨 마지막 물체의 Last부터 avoid 끝나는 곳까지 맵 넣기
    st_AvoidDirection.f32_X = 0;
    st_AvoidDirection.f32_Y = 0;
    f32_AvoidDist = 0;
    // MakePath_LiDAR(st_Planning, ps_ObjectFirstLastIdx[st_LiDAR.s32_ObjectNum - 1].second, st_Planning.s32_RefPathEndIdx, st_AvoidDirection, f32_AvoidDist);
    MakePath_LiDAR(st_Planning, s32_AvoidLastIdx, st_Planning.s32_RefPathEndIdx, st_AvoidDirection, f32_AvoidDist);
    if (s32_StartLocalIdx < st_Planning.s32_GlobalNearIdx)
    {
        st_Planning.s32_LocalNearIdx = st_Planning.s32_GlobalNearIdx - s32_StartLocalIdx;
    }
    // std::cout << "-------------------------------LocalPath---------------------------" << std::endl;
    // for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i++)
    // {
    //     std::cout << "(" << st_Planning.st_LocalPath.arf32_E[s32_i] << ", " << st_Planning.st_LocalPath.arf32_N[s32_i] << ")" << std::endl;
    // }
    // std::cout << "(" << st_Planning.st_LocalPath.arf32_E[0] << ", " << st_Planning.st_LocalPath.arf32_N[0] << ")" << std::endl;
}

void Avoid_LiDAR(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Avoid_t &st_AvoidPara)
{

    // cout << "Avoid Start " << endl;
    if (st_LiDAR.s32_ObjectNum == 0)
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        cout << "no object" << endl;
        return;
    }
    // 장애물 없을 때 global 주행할 수 있도록 or global 그대로 local에 넣기 (예외처리)
    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        std::cout << "(" << st_LiDAR.st_XY[s32_i].f32_X << ", " << st_LiDAR.st_XY[s32_i].f32_Y << ")" << std::endl;
    }
    std::vector<std::vector<std::vector<float32_t>>> vf_PerpendicularPoints(st_LiDAR.s32_ObjectNum, std::vector<std::vector<float32_t>>(1, std::vector<float32_t>(2, 0)));
    // std::vector<bool> b_ObjectDivision(st_LiDAR.s32_ObjectNum, 0);
    bool b_ObjectDivision[100] = {0};
    Point2D_t st_OptimalPoints[st_LiDAR.s32_ObjectNum] = {0};
    std::pair<int32_t, int32_t> ps_ObjectFirstLastIdx[st_LiDAR.s32_ObjectNum];
    std::vector<std::vector<int32_t>> vs_LinkedList(st_LiDAR.s32_ObjectNum, std::vector<int32_t>(1, 0));
    int32_t s32_LinkedObjectNum = 0;
    st_Planning.s32_LocalNearIdx = 0;
    bool b_OptimalExist = true;

    // MakeRefPath(st_Planning, st_GPSIMU);

    FindObjectNear_LiDAR(st_LiDAR, st_Planning);
    // 감지된 장애물 모두 near점 찾기 (global에서 위치 찾기)

    Perpendicular_LiDAR(st_Planning, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum,
                  st_AvoidPara.s32_Iter, st_AvoidPara.f32_Gap, vf_PerpendicularPoints);
    // 장애물 near idx에서 수직점 만들어주기 (Perpendicular)


    Constraint_LiDAR(st_Planning, st_LiDAR.st_XY, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum,
                vf_PerpendicularPoints, st_AvoidPara.f32_Thres, st_AvoidPara.f32_LeftThres, st_AvoidPara.f32_RightThres);
    // 수직점들 중 물체와 가까운, boundary 밖에 점 제거 (Constraint)

    CostFunction_LiDAR(st_GPSIMU, st_Planning, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum, st_LiDAR.st_XY,
                 vf_PerpendicularPoints, st_OptimalPoints, b_OptimalExist, st_AvoidPara.arf32_Costs);
    // cost function을 통해 수직점들 중 optimal point 선택 (CostFunction)

    if (b_OptimalExist == false)
    {
        b_SlowSignal = true;
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        std::cout << "Not Optimal Point" << std::endl;
        return;
    }
    // optimal 점이 없고 회피할 경로가 없을 경우 slow (예외처리)

    SelectFirstLastPoint_LiDAR(st_Planning, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum,
                         st_AvoidPara.f32_FirstDist, st_AvoidPara.f32_LastDist, ps_ObjectFirstLastIdx);
    // 평행이동 할 idx 찾기 - first, last

    ObjectLinked_LiDAR(st_LiDAR, ps_ObjectFirstLastIdx, vs_LinkedList, s32_LinkedObjectNum);
    // 여러 물체 간의 first, last 관계를 통해 겹치는 곳 예외처리 (ObjectLinked)

    MakeAvoidPath_LiDAR(st_LiDAR, st_Planning, st_OptimalPoints, ps_ObjectFirstLastIdx, vs_LinkedList, s32_LinkedObjectNum);
    // Linked된 물체들을 통해 최종적인 avoid path 생성 (MakeAvoidPath)

	// VisualizeRefPath(Color::GREEN, 0.5f, st_Planning, st_GPSIMU);
    VisualizeLocalPath_Avoid(Color::RED, 0.5f, st_Planning);
    // VisualizePoint({0.0, 0.0}, Color::ORANGE, 1.5f);
}
