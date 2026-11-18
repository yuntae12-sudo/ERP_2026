#include <Planning/Planning.hpp>
extern int32_t s32_StartIdx, s32_EndIdx;
extern bool b_SlowSignal, b_StaySignal;

using namespace std;

void FindObjectNear(LiDAR_t &st_LiDAR, Planning_t &st_Planning)
{
    int32_t s32_NearLen = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second -
                          st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;

    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        int32_t s32_ObjectIdx = FindNear(st_Planning, st_LiDAR.st_EN[s32_i], st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first, s32_NearLen, "Global");
        // 이전 물체와 너무 가까우면 임의로 idx 10 정도 떨어트려서 near점 넣어주기 (1m)
        if (s32_i > 0 && abs(st_LiDAR.ars32_ObjectNearIdx[s32_i] - s32_ObjectIdx) < 10)
        {
            st_LiDAR.ars32_ObjectNearIdx[s32_i] = st_LiDAR.ars32_ObjectNearIdx[s32_i - 1] + 10;
        }
        else
        {
            st_LiDAR.ars32_ObjectNearIdx[s32_i] = s32_ObjectIdx;
        }
    }
}

void Perpendicular(Planning_t &st_Planning, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                   int32_t &s32_Iter, float32_t &f32_Gap,
                   std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints, std::string s_PathType)
{
    Path_t st_Path;
    Point2D_t st_UnitVector = {0}, st_LeftDirection = {0}, st_RightDirection = {0};
    std::vector<float32_t> vf_ObjectNearPoint(2, 0);
    if (s_PathType == "Global")
    {
        st_Path = st_Planning.st_GlobalPath;
    }
    else if (s_PathType == "Local")
    {
        st_Path = st_Planning.st_LocalPath;
    }
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        // Tangent Vector get
        if (ars32_ObjectNearIdx[s32_i] < st_Path.s32_PathNum - 1)
        {
            st_UnitVector = UnitVector(st_Path.arf32_E[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_N[ars32_ObjectNearIdx[s32_i]],
                                       st_Path.arf32_E[ars32_ObjectNearIdx[s32_i] + 1], st_Path.arf32_N[ars32_ObjectNearIdx[s32_i] + 1]);
        }
        else
        {
            st_UnitVector = UnitVector(st_Path.arf32_E[ars32_ObjectNearIdx[s32_i] - 1], st_Path.arf32_N[ars32_ObjectNearIdx[s32_i] - 1],
                                       st_Path.arf32_E[ars32_ObjectNearIdx[s32_i]], st_Path.arf32_N[ars32_ObjectNearIdx[s32_i]]);
        }
        // +-Normal Vector get
        st_LeftDirection = RotatePoint(st_UnitVector, -90);
        st_RightDirection = RotatePoint(st_UnitVector, 90);

        // Cal Perpendicular Points
        vf_PerpendicularPoints[s32_i].push_back({st_Path.arf32_E[ars32_ObjectNearIdx[s32_i]],
                                                 st_Path.arf32_N[ars32_ObjectNearIdx[s32_i]]}); // ok
        vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin());
        vf_ObjectNearPoint[0] = st_Path.arf32_E[ars32_ObjectNearIdx[s32_i]];
        vf_ObjectNearPoint[1] = st_Path.arf32_N[ars32_ObjectNearIdx[s32_i]];
        
        GetGapPoints(st_Planning, vf_ObjectNearPoint, st_LeftDirection, s32_Iter, f32_Gap, vf_PerpendicularPoints[s32_i]);
        GetGapPoints(st_Planning, vf_ObjectNearPoint, st_RightDirection, s32_Iter, f32_Gap, vf_PerpendicularPoints[s32_i]);
        // std::cout << "Perpendicular -------------------" << std::endl;
        // for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j ++)
        // {
        //     cout << "(" << vf_PerpendicularPoints[s32_i][s32_j][0] << ", " << vf_PerpendicularPoints[s32_i][s32_j][1] << ")" << endl;
        // }
    }
}

void ObjectDivision(Planning_t &st_Planning, LiDAR_t &st_LiDAR, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, 
                    bool *b_ObjectDivision, std::string s_PathType)
{
    Path_t st_Path;

    std::vector<float32_t> vf_ObjectNearPoint(2, 0);
    std::vector<float32_t> vf_ObjectyMinPoint(2, 0);
    std::vector<float32_t> vf_ObjectyMaxPoint(2, 0);
    if (s_PathType == "Global")
    {
        st_Path = st_Planning.st_GlobalPath;
    }
    else if (s_PathType == "Local")
    {
        st_Path = st_Planning.st_LocalPath;
    }
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        vf_ObjectNearPoint[0] = st_Path.arf32_E[ars32_ObjectNearIdx[s32_i]];
        vf_ObjectNearPoint[1] = st_Path.arf32_N[ars32_ObjectNearIdx[s32_i]];
        vf_ObjectyMinPoint[0] = st_LiDAR.st_ENyMin[s32_i].f32_X;
        vf_ObjectyMinPoint[1] = st_LiDAR.st_ENyMin[s32_i].f32_Y;
        vf_ObjectyMaxPoint[0] = st_LiDAR.st_ENyMax[s32_i].f32_X;
        vf_ObjectyMaxPoint[1] = st_LiDAR.st_ENyMax[s32_i].f32_Y;

        float32_t YMinDist = Normalize2D(vf_ObjectNearPoint[0], vf_ObjectNearPoint[1], vf_ObjectyMinPoint[0], vf_ObjectyMinPoint[1]);
        float32_t YMaxDist = Normalize2D(vf_ObjectNearPoint[0], vf_ObjectNearPoint[1], vf_ObjectyMaxPoint[0], vf_ObjectyMaxPoint[1]);

        if(YMinDist > YMaxDist)
        {
            b_ObjectDivision[s32_i] = false; // Object locate Right Side
        }
        else
        {
            b_ObjectDivision[s32_i] = true; // Object locate Left Side
        }

    }

}

void Constraint(Planning_t &st_Planning, Point2D_t *st_EN, int32_t s32_ObjectNum, std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints, float32_t f32_Thres)
{
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        std::cout << "Constraint -------------------" << std::endl;
        for (int32_t s32_j = vf_PerpendicularPoints[s32_i].size() - 1; s32_j >= 0; s32_j--)
        {
            if (!InOrOut(st_Planning, vf_PerpendicularPoints[s32_i][s32_j][0], vf_PerpendicularPoints[s32_i][s32_j][1]) ||
                Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_EN[s32_i]) < f32_Thres)
            {
                vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            }
            // if (!InOrOut(st_Planning, vf_PerpendicularPoints[s32_i][s32_j][0], vf_PerpendicularPoints[s32_i][s32_j][1]))
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
            // if (Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_EN[s32_i]) < f32_Thres)
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
        }
        // for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j++)
        // {
        //     cout << "(" << vf_PerpendicularPoints[s32_i][s32_j][0] << ", " << vf_PerpendicularPoints[s32_i][s32_j][1] << ")" << endl;
        // }
    }
}

void Constraint(GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, LiDAR_t &st_LiDAR, Point2D_t *st_EN, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                bool *b_ObjectDivision, std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                float32_t f32_Thres, std::string s_PathType)
{
    Path_t st_Path;
    Point2D_t st_ObjectUnitVector = {0}, st_PerUnitvector = {0};

    if (s_PathType == "Global")
    {
        st_Path = st_Planning.st_GlobalPath;
    }
    else if (s_PathType == "Local")
    {
        st_Path = st_Planning.st_LocalPath;
    }

    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        ObjectDivision(st_Planning, st_LiDAR, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum, b_ObjectDivision, "Global");

        std::cout << "Constraint -------------------" << std::endl;
        for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size();)
        {
            st_ObjectUnitVector = UnitVector(st_GPSIMU.f32_E, st_GPSIMU.f32_N, st_LiDAR.st_EN[s32_i].f32_X, st_LiDAR.st_EN[s32_i].f32_Y);
            st_PerUnitvector = UnitVector(st_GPSIMU.f32_E, st_GPSIMU.f32_N, vf_PerpendicularPoints[s32_i][s32_j][0],vf_PerpendicularPoints[s32_i][s32_j][1]);
            if (!InOrOut(st_Planning, vf_PerpendicularPoints[s32_i][s32_j][0], vf_PerpendicularPoints[s32_i][s32_j][1]) ||
                Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_EN[s32_i]) < f32_Thres)
            {
                vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            }
            // if (!InOrOut(st_Planning, vf_PerpendicularPoints[s32_i][s32_j][0], vf_PerpendicularPoints[s32_i][s32_j][1]))
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
            // if (Normalize2D(vf_PerpendicularPoints[s32_i][s32_j], st_EN[s32_i]) < f32_Thres)
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
            // if ((b_ObjectDivision[s32_i] == 1 && CrossProduct(st_ObjectUnitVector, st_PerUnitvector) == 1) ||
            //     (b_ObjectDivision[s32_i] == 0 && CrossProduct(st_ObjectUnitVector, st_PerUnitvector) == 0))
            // {
            //     vf_PerpendicularPoints[s32_i].erase(vf_PerpendicularPoints[s32_i].begin() + s32_j);
            // }
            else
            {
                 s32_j++;
            }
        }
        for (int32_t s32_j = 0; s32_j < vf_PerpendicularPoints[s32_i].size(); s32_j++)
        {
            cout << "(" << vf_PerpendicularPoints[s32_i][s32_j][0] << ", " << vf_PerpendicularPoints[s32_i][s32_j][1] << ")" << endl;
        }
    }
}

void CostFunction(GPSIMU_t &st_GPSIMU, Planning_t &st_Planning,
                  int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, Point2D_t *st_EN,
                  std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                  Point2D_t *st_OptimalPoints, bool &b_SlowSignal,
                  float32_t *arf_Costs)
{
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
            b_SlowSignal = true;
            st_OptimalPoints[s32_i] = {st_Planning.st_GlobalPath.arf32_E[ars32_ObjectNearIdx[s32_i]],
                                       st_Planning.st_GlobalPath.arf32_N[ars32_ObjectNearIdx[s32_i]]};
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
                                                              st_EN[s32_i]);
            // cout << "-------------------------------Dbject----------------------" << endl;
            // cout << "st_EN[0]: " << st_EN[s32_i].f32_X << ", " << "st_EN[1]: " << st_EN[s32_i].f32_Y << endl;
            // 장애물이랑 포인트랑에 거리 --> 포인트랑 장애물이랑 멀수록 크기가 커져 --> optimal을 뽑을 때는 최소인 애를 구하는거야 --> 그래서 cost가 -??

            arf_Path_PerpendPointsDist[s32_j] = Normalize2D(vf_PerpendicularPoints[s32_i][s32_j],
                                                            st_Planning.st_GlobalPath.arf32_E[ars32_ObjectNearIdx[s32_i]],
                                                            st_Planning.st_GlobalPath.arf32_N[ars32_ObjectNearIdx[s32_i]]);
            // 포인트랑 경로 상의 장애물 니어랑 거리 --> 니어점을 기준으로 수직점을 찍어준 포인트랑 니어점이랑 거리 --> 거리랑 회피 경로 간의 거리 --> global 경로랑 안 떨어지겠다 --> cost +

            arf_Vehicle_PerpendPointsDist[s32_j] = Normalize2D(vf_PerpendicularPoints[s32_i][s32_j],
                                                               st_GPSIMU.f32_E, st_GPSIMU.f32_N);
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
        std::cout << "Optimal -------------------------------" << std::endl;
        std::cout << "(" << st_OptimalPoints[s32_i].f32_X << ", " << st_OptimalPoints[s32_i].f32_Y << ")" << std::endl;
    }
}

void SelectFirstLastPoint(Planning_t &st_Planning,
                          int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, float32_t &f32_FirstDist, float32_t &f32_LastDist,
                          std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx)
{
    int32_t s32_NearLen = s32_EndIdx - s32_StartIdx;
    for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    {
        ps_ObjectFirstLastIdx[s32_i].first = s32_StartIdx;
        ps_ObjectFirstLastIdx[s32_i].second = s32_EndIdx;
        for (int32_t s32_j = s32_StartIdx; s32_j < ars32_ObjectNearIdx[s32_i]; s32_j++)
        {
            // Object의 near에서부터 ~m 떨어진 지점
            if (Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N, s32_j, ars32_ObjectNearIdx[s32_i]) < f32_FirstDist)
            {
                ps_ObjectFirstLastIdx[s32_i].first = s32_j;
                break;
            }
        }
        for (int32_t s32_j = ars32_ObjectNearIdx[s32_i]; s32_j < s32_EndIdx; s32_j++)
        {
            if (Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N, s32_j, ars32_ObjectNearIdx[s32_i]) > f32_LastDist)
            {
                ps_ObjectFirstLastIdx[s32_i].second = s32_j;
                break;
            }
        }
    }
    std::cout << "Map----------------------------------------" << std::endl;
    for (int32_t s32_i = s32_StartIdx; s32_i < s32_EndIdx; s32_i++)
    {
        std::cout << "(" << st_Planning.st_GlobalPath.arf32_E[s32_i] << ", " << st_Planning.st_GlobalPath.arf32_N[s32_i] << ")" << std::endl;
    }
    // std::cout << "FirstLast--------------------------------------" << std::endl;
    // for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
    // {
        // std::cout << "(" << st_Planning.st_GlobalPath.arf32_E[ps_ObjectFirstLastIdx[s32_i].first] << ", " << st_Planning.st_GlobalPath.arf32_N[ps_ObjectFirstLastIdx[s32_i].first] << ")" << std::endl;
        // std::cout << "(" << st_Planning.st_GlobalPath.arf32_E[ps_ObjectFirstLastIdx[s32_i].second] << ", " << st_Planning.st_GlobalPath.arf32_N[ps_ObjectFirstLastIdx[s32_i].second] << ")" << std::endl;
    // }
}

void ObjectLinked(LiDAR_t &st_LiDAR,
                  std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx, std::vector<std::vector<int32_t>> &vs_LinkedList, int32_t &s32_LinkedObjectNum)
{
    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        // 처음은 무조건 넣고
        if (s32_LinkedObjectNum == 0)
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

void MakeAvoidPath(LiDAR_t &st_LiDAR, Planning_t &st_Planning,
                   Point2D_t *st_OptimalPoints, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx,
                   std::vector<std::vector<int32_t>> &vs_LinkedList, int32_t &s32_LinkedObjectNum)
{
    int32_t s32_AvoidStartIdx = 0, s32_AvoidLastIdx = 0;
    Point2D_t st_AvoidDirection = {0};
    float32_t f32_AvoidDist = 0;
    int32_t s32_StartLocalIdx = st_Planning.s32_GlobalNearIdx;
    st_Planning.st_LocalPath.s32_PathNum = 0;

    // 현재 Near부터 첫번째 물체의 first까지 맵 넣기
    MakePath(st_Planning, st_Planning.s32_GlobalNearIdx, ps_ObjectFirstLastIdx[0].first, st_AvoidDirection, f32_AvoidDist);
    // Linked된 물체에 대해서 값 넣기
    for (int32_t s32_i = 0; s32_i < s32_LinkedObjectNum; s32_i++)
    {
        if (s32_i != 0)
        {
            // Linked된 물체들 중 이전의 맨 마지막과 현재의 맨 처음까지 이어주기
            st_AvoidDirection.f32_X = 0;
            st_AvoidDirection.f32_Y = 0;
            f32_AvoidDist = 0;
            MakePath(st_Planning, ps_ObjectFirstLastIdx[vs_LinkedList[s32_i - 1].back()].second,
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
            st_AvoidDirection = UnitVector(st_Planning.st_GlobalPath.arf32_E[st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]]],
                                           st_Planning.st_GlobalPath.arf32_N[st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]]],
                                           st_OptimalPoints[vs_LinkedList[s32_i][s32_j]].f32_X, st_OptimalPoints[vs_LinkedList[s32_i][s32_j]].f32_Y);

            f32_AvoidDist = Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N, st_LiDAR.ars32_ObjectNearIdx[vs_LinkedList[s32_i][s32_j]],
                                        st_OptimalPoints[vs_LinkedList[s32_i][s32_j]]);
            // std::cout << "f32_AvoidDist : " << f32_AvoidDist << std::endl;
            // nan 뜨는거 예외처리
            if (isnan(st_AvoidDirection.f32_X) || isnan(st_AvoidDirection.f32_Y))
            {
                st_AvoidDirection.f32_X = 0;
                st_AvoidDirection.f32_Y = 0;
            }
            // 첫번째 Linked 물체 중에 1개면
            if (vs_LinkedList[s32_i].size() == 1)
            {
                MakePath(st_Planning, s32_AvoidStartIdx, s32_AvoidLastIdx, st_AvoidDirection, f32_AvoidDist);
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
            MakePath(st_Planning, s32_AvoidStartIdx, s32_AvoidLastIdx, st_AvoidDirection, f32_AvoidDist);
        }
    }
    // 맨 마지막 물체의 Last부터 avoid 끝나는 곳까지 맵 넣기
    st_AvoidDirection.f32_X = 0;
    st_AvoidDirection.f32_Y = 0;
    f32_AvoidDist = 0;
    MakePath(st_Planning, ps_ObjectFirstLastIdx[st_LiDAR.s32_ObjectNum - 1].second, s32_EndIdx, st_AvoidDirection, f32_AvoidDist);
    if (s32_StartLocalIdx < st_Planning.s32_GlobalNearIdx)
    {
        st_Planning.s32_LocalNearIdx = st_Planning.s32_GlobalNearIdx - s32_StartLocalIdx;
    }
    std::cout << "-------------------------------LocalPath---------------------------" << std::endl;
    for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i++)
    {
        std::cout << "(" << st_Planning.st_LocalPath.arf32_E[s32_i] << ", " << st_Planning.st_LocalPath.arf32_N[s32_i] << ")" << std::endl;
    }
    // std::cout << "(" << st_Planning.st_LocalPath.arf32_E[0] << ", " << st_Planning.st_LocalPath.arf32_N[0] << ")" << std::endl;
}

void Avoid(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Avoid_t &st_AvoidPara)
{

    // cout << "Avoid Start " << endl;
    cout << "st_LiDAR.s32_ObjectNum: " << st_LiDAR.s32_ObjectNum << endl;
    if (st_LiDAR.s32_ObjectNum == 0)
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        cout << "no object" << endl;
        return;
    }
    // 장애물 없을 때 global 주행할 수 있도록 or global 그대로 local에 넣기 (예외처리)
    // std::cout << "-----------------------obs" << std::endl;
    // for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    // {
    //     std::cout << "(" << st_LiDAR.st_EN[s32_i].f32_X << ", " << st_LiDAR.st_EN[s32_i].f32_Y << ")" << std::endl;
    // }
    s32_StartIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;
    s32_EndIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second;
    std::vector<std::vector<std::vector<float32_t>>> vf_PerpendicularPoints(st_LiDAR.s32_ObjectNum, std::vector<std::vector<float32_t>>(1, std::vector<float32_t>(2, 0)));
    // std::vector<bool> b_ObjectDivision(st_LiDAR.s32_ObjectNum, 0);
    bool b_ObjectDivision[100] = {0};
    Point2D_t st_OptimalPoints[st_LiDAR.s32_ObjectNum] = {0};
    std::pair<int32_t, int32_t> ps_ObjectFirstLastIdx[st_LiDAR.s32_ObjectNum];
    std::vector<std::vector<int32_t>> vs_LinkedList(st_LiDAR.s32_ObjectNum, std::vector<int32_t>(1, 0));
    int32_t s32_LinkedObjectNum = 0;
    st_Planning.s32_LocalNearIdx = 0; 

    FindObjectNear(st_LiDAR, st_Planning);
    // 감지된 장애물 모두 near점 찾기 (global에서 위치 찾기)

    Perpendicular(st_Planning, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum,
                  st_AvoidPara.s32_Iter, st_AvoidPara.f32_Gap, vf_PerpendicularPoints, "Global");
    // 장애물 near idx에서 수직점 만들어주기 (Perpendicular)


    Constraint(st_GPSIMU, st_Planning, st_LiDAR, st_LiDAR.st_EN, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum,
                b_ObjectDivision, vf_PerpendicularPoints, st_AvoidPara.f32_Thres, "Global");
    // 수직점들 중 물체와 가까운, boundary 밖에 점 제거 (Constraint)

    CostFunction(st_GPSIMU, st_Planning, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum, st_LiDAR.st_EN,
                 vf_PerpendicularPoints, st_OptimalPoints, b_SlowSignal, st_AvoidPara.arf32_Costs);
    // cost function을 통해 수직점들 중 optimal point 선택 (CostFunction)

    if (b_SlowSignal == true)
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        std::cout << "Not Optimal Point" << std::endl;
        return;
    }
    // optimal 점이 없고 회피할 경로가 없을 경우 slow (예외처리)

    SelectFirstLastPoint(st_Planning, st_LiDAR.ars32_ObjectNearIdx, st_LiDAR.s32_ObjectNum,
                         st_AvoidPara.f32_FirstDist, st_AvoidPara.f32_LastDist, ps_ObjectFirstLastIdx);
    // 평행이동 할 idx 찾기 - first, last

    ObjectLinked(st_LiDAR, ps_ObjectFirstLastIdx, vs_LinkedList, s32_LinkedObjectNum);
    // 여러 물체 간의 first, last 관계를 통해 겹치는 곳 예외처리 (ObjectLinked)

    MakeAvoidPath(st_LiDAR, st_Planning, st_OptimalPoints, ps_ObjectFirstLastIdx, vs_LinkedList, s32_LinkedObjectNum);
    // Linked된 물체들을 통해 최종적인 avoid path 생성 (MakeAvoidPath)

    VisualizeLocalPath(Color::GREEN, 0.5f, st_Planning, st_GPSIMU);
    VisualizePoint({0.0, 0.0}, Color::ORANGE, 1.5f);
    std::cout << "-----------------------------pos" << std::endl;
    std::cout << "(" << st_GPSIMU.f32_E << "," << st_GPSIMU.f32_N << ")" << std::endl;

    std::cout << "--------------------------Boundary" << std::endl;
    Path_t arst_Paths[4] = {
        st_Planning.st_InBoundary2,
        st_Planning.st_OutBoundary
    };
    int32_t ars32_NearIdxs[2] = {0};
    for (int32_t s32_i = 0; s32_i<2; s32_i++)
    {
        float32_t arf_Dist[arst_Paths[s32_i].s32_PathNum] = {0};
        for (int32_t s32_j = 0; s32_j < arst_Paths[s32_i].s32_PathNum; s32_j++)
        {
            arf_Dist[s32_j] = Normalize2D(arst_Paths[s32_i].arf32_E, arst_Paths[s32_i].arf32_N, s32_j % arst_Paths[s32_i].s32_PathNum, st_GPSIMU.f32_E, st_GPSIMU.f32_N);
        }
        ars32_NearIdxs[s32_i] = std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + arst_Paths[s32_i].s32_PathNum)) % arst_Paths[s32_i].s32_PathNum;
        for (int32_t s32_j = std::max(ars32_NearIdxs[s32_i] - 100, 0); s32_j < std::min(ars32_NearIdxs[s32_i] + 100, arst_Paths[s32_i].s32_PathNum); s32_j++)
        {
            std::cout << "(" << arst_Paths[s32_i].arf32_E[s32_j] << "," << arst_Paths[s32_i].arf32_N[s32_j] << ")" << std::endl;
        }
    }
    std::cout << "--------------------------LiDAR" << std::endl;
    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        std::cout << "(" << st_LiDAR.st_EN[s32_i].f32_X << "," << st_LiDAR.st_EN[s32_i].f32_Y << ")" << std::endl;
    }
}