#include <Planning/Planning.hpp>

extern int32_t s32_StartIdx, s32_EndIdx;

Point2D_t st_FilterLiDARObject[c_LIDARMAXSIZE] = {0};
int32_t s32_FilterLiDARObjectCnt;
float32_t f32_PointXMargin = 1.5;
float32_t f32_PointYMargin = 2.5;

void MakeUTurnPath(Planning_t &st_Planning) {
    st_Planning.st_LocalPath.s32_PathNum = 0;
    int32_t s32_PointsNum= 10;

    for (int32_t s32_i = 0; s32_i < s32_FilterLiDARObjectCnt; s32_i++) {
        Point2D_t st_StartPoint = {st_FilterLiDARObject[s32_i].f32_X + f32_PointXMargin,
                                   st_FilterLiDARObject[s32_i].f32_Y + f32_PointYMargin};

        Point2D_t st_EndPoint = {st_FilterLiDARObject[s32_i + 1].f32_X + f32_PointXMargin,
                                 st_FilterLiDARObject[s32_i + 1].f32_Y + f32_PointYMargin};

        for (int32_t s32_j = 0; s32_j < s32_PointsNum; s32_j++) {
            float32_t f32_Tmp = s32_j /s32_PointsNum;
            float32_t f32_InterpolatedX = st_StartPoint.f32_X + f32_Tmp * (st_EndPoint.f32_X - st_StartPoint.f32_X);
            float32_t f32_InterpolatedY = st_StartPoint.f32_Y + f32_Tmp * (st_EndPoint.f32_Y - st_StartPoint.f32_Y);

            st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedX;
            st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedY;
            st_Planning.st_LocalPath.s32_PathNum++;
        }
    }
    VisualizeLocalPath_Avoid(Color::RED, 0.5f, st_Planning);
    cout << "st_Planning.st_LocalPath.s32_PathNum : " << st_Planning.st_LocalPath.s32_PathNum << endl;
}


void Uturn_LiDAR(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Uturn_t &st_Uturn)
{
    s32_FilterLiDARObjectCnt = 0;

    float32_t f32_StartIdx_PosDistance = 0, f32_EndIdx_PosDistance = 0;
    Point2D_t st_Direction = {0};
    float32_t f32_Gap = 0;
    s32_StartIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;
    s32_EndIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second;
    st_Planning.s32_GlobalNearIdx = FindNear(st_Planning, st_GPSIMU.f32_E, st_GPSIMU.f32_N, 4000, "Global");
    cout << "--------------------------------------------------------" << endl;
    cout << "st_Planning.s32_GlobalNearIdx : " << st_Planning.s32_GlobalNearIdx << endl;
    if (st_LiDAR.s32_ObjectNum <= 1 || st_Planning.s32_GlobalNearIdx > 4500)
    {
        // 감지된 Obeject가 없고 지금까지 Uturn 내에서 선택된 ConnectObject가 없다면
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        return;
    }
    if (st_LiDAR.s32_ObjectNum > 1)
    {
        for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
        {
            if(st_LiDAR.st_XY[s32_i].f32_Y < 0)
            {
                s32_FilterLiDARObjectCnt++;
                st_FilterLiDARObject[s32_i] = st_LiDAR.st_XY[s32_i];
            }
        }
        if(s32_FilterLiDARObjectCnt >= 2)
        {
            MakeUTurnPath(st_Planning);
        }
        else if (s32_FilterLiDARObjectCnt < 2)
        {
            st_Planning.st_Mission.s_SelectName = "TRACKING";
        }
    }
}