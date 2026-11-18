#include <Planning/Planning.hpp>

extern int32_t s32_StartIdx, s32_EndIdx;

extern float32_t f32_MoraiENUYaw;

int32_t ars32_ObjectIdx[1] = {-1};
Point2D_t st_LastObject = {0};

Point2D_t st_FilterObject[c_LIDARMAXSIZE] = {0};
int32_t s32_FilterObjectCnt;

int32_t s32_UturnFindIdxLen = 4000;
int32_t s32_FirstIdx = 0, s32_LastIdx = 0;
bool b_UturnPathMakeFlag = false;
int32_t s32_EndGlobalNearIdx = 0;
bool b_Local_Global = false;

Point2D_t FindCircleCenter(const Point2D_t& P1, const Point2D_t& P2, const Point2D_t& P3) {
    float32_t A = P1.f32_X * P1.f32_X + P1.f32_Y * P1.f32_Y;
    float32_t B = P2.f32_X * P2.f32_X + P2.f32_Y * P2.f32_Y;
    float32_t C = P3.f32_X * P3.f32_X + P3.f32_Y * P3.f32_Y;

    float32_t D = 2 * (P1.f32_X * (P2.f32_Y - P3.f32_Y) + P2.f32_X * (P3.f32_Y - P1.f32_Y) + P3.f32_X * (P1.f32_Y - P2.f32_Y));

    float32_t f32_CenterX = ((A * (P2.f32_Y - P3.f32_Y) + B * (P3.f32_Y - P1.f32_Y) + C * (P1.f32_Y - P2.f32_Y)) / D);
    float32_t f32_CenterY = ((A * (P3.f32_X - P2.f32_X) + B * (P1.f32_X - P3.f32_X) + C * (P2.f32_X - P1.f32_X)) / D);

    return {f32_CenterX, f32_CenterY};
}

float32_t NED2ENU(float32_t f32_NEDHeading) {
    float32_t f32_ENUHeading = 90 - f32_NEDHeading;

    if (f32_ENUHeading > 180)
    {
        f32_ENUHeading -= 360;
    } 
    else if (f32_ENUHeading < -180)
    {
        f32_ENUHeading += 360;
    }
    return Deg2Rad(f32_ENUHeading);
}

void ConnectObject(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, float32_t &f32_UturnAngleThreshold_deg)
{
    float32_t f32_MaxAngle_rad = 0, f32_MinAngle_rad = 0, f32_ObjectAngle_rad = 0;
    // cout << "f32_RefAngle_rad : " << f32_RefAngle_rad << endl;
    float32_t f32_AngleThreshold_rad = Deg2Rad(f32_UturnAngleThreshold_deg);
    float32_t f32_RefAngle_rad = 0;
    
    if (f32_MoraiENUYaw != 0)
    {
        f32_RefAngle_rad = Deg2Rad(f32_MoraiENUYaw);
    }
    else if (f32_MoraiENUYaw == 0)
    {
        f32_RefAngle_rad = NED2ENU(st_GPSIMU.f32_Heading_deg);
    }

    for (int32_t s32_i = 0; s32_i < s32_FilterObjectCnt - 1; s32_i++)
    {
        f32_MaxAngle_rad = f32_RefAngle_rad + f32_AngleThreshold_rad;
        f32_MinAngle_rad = f32_RefAngle_rad - f32_AngleThreshold_rad;
        f32_ObjectAngle_rad = atan2(st_FilterObject[s32_i + 1].f32_Y - st_FilterObject[s32_i].f32_Y,
                                    st_FilterObject[s32_i + 1].f32_X - st_FilterObject[s32_i].f32_X);
        // cout << "f32_ObjectAngle_rad : " << f32_ObjectAngle_rad << endl;

        // 탐색하고자 하는 Angle 범위 넘는 애들 처리
        if (f32_ObjectAngle_rad < f32_MinAngle_rad || f32_MaxAngle_rad < f32_ObjectAngle_rad)
        {
            st_LastObject = st_FilterObject[s32_i + 1];
            int32_t s32_LastObjectIdx = FindNear(st_Planning, st_LastObject, s32_StartIdx, 1000, "Global");
            ars32_ObjectIdx[0] = s32_LastObjectIdx;
            // std::cout << "s32_LastObjectIdx : " << s32_LastObjectIdx << std::endl;
            break;
        }
    }
}

void UturnFindObjectNear(LiDAR_t &st_LiDAR, Planning_t &st_Planning, int32_t &s32_Iter, float32_t &f32_Gap)
{
    int32_t s32_NearLen = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second -
                          st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;
    int32_t s32_MinIdx = 99999, s32_MaxIdx = 0;
    
    std::vector<std::vector<std::vector<float32_t>>> vf_PerpendicularPoints(1, std::vector<std::vector<float32_t>>(1, std::vector<float32_t>(2, 0)));
    Perpendicular(st_Planning, ars32_ObjectIdx, 1, s32_Iter, f32_Gap, vf_PerpendicularPoints, "Global");

    for (int32_t s32_i = 0; s32_i < vf_PerpendicularPoints[0].size(); s32_i++)
    {
        for (int32_t s32_j = st_Planning.s32_GlobalNearIdx; s32_j < st_Planning.s32_GlobalNearIdx + s32_UturnFindIdxLen; s32_j++)
        {
            // Uturn 시작과 끝 찾기
            if (Normalize2D(vf_PerpendicularPoints[0][s32_i][0], vf_PerpendicularPoints[0][s32_i][1],
                            st_Planning.st_GlobalPath.arf32_E[s32_j], st_Planning.st_GlobalPath.arf32_N[s32_j]) < 2)
            {
                if (s32_MinIdx > s32_j)
                    s32_MinIdx = s32_j;
                if (s32_MaxIdx < s32_j)
                    s32_MaxIdx = s32_j;
            }
        }
    }
    s32_FirstIdx = s32_MinIdx;
    s32_LastIdx = s32_MaxIdx;
}

void MakeLinearPath(Planning_t &st_Planning, int32_t &s32_UturnIdxMargin)
{
    Point2D_t st_XY1_tmp = {st_Planning.st_GlobalPath.arf32_E[s32_FirstIdx + s32_UturnIdxMargin],
                        st_Planning.st_GlobalPath.arf32_N[s32_FirstIdx + s32_UturnIdxMargin]};
    Point2D_t st_XY2_tmp = {st_Planning.st_GlobalPath.arf32_E[s32_LastIdx - s32_UturnIdxMargin],
                        st_Planning.st_GlobalPath.arf32_N[s32_LastIdx - s32_UturnIdxMargin]};

    Point2D_t st_XY1 = {st_Planning.st_GlobalPath.arf32_E[s32_FirstIdx],
                        st_Planning.st_GlobalPath.arf32_N[s32_FirstIdx]};
    
    Point2D_t st_XY2 = {(st_XY1_tmp.f32_X + st_XY2_tmp.f32_X) / 2, (st_XY1_tmp.f32_Y + st_XY2_tmp.f32_Y) / 2};

    Point2D_t st_XY3 = {st_Planning.st_GlobalPath.arf32_E[s32_LastIdx],
                        st_Planning.st_GlobalPath.arf32_N[s32_LastIdx]};
    
    Point2D_t st_CircleCenterPoint = FindCircleCenter(st_XY1, st_XY2, st_XY3);
    float32_t f32_Radius = Normalize2D(st_XY1, st_CircleCenterPoint);

    float32_t f32_StartAngle = atan2(st_XY1.f32_Y - st_CircleCenterPoint.f32_Y, st_XY1.f32_X - st_CircleCenterPoint.f32_X);
    float32_t f32_EndAngle = atan2(st_XY3.f32_Y - st_CircleCenterPoint.f32_Y, st_XY3.f32_X - st_CircleCenterPoint.f32_X);


    st_Planning.st_LocalPath.s32_PathNum = 0;
    int32_t s32_NumPoints = 100; // 생성할 점의 수
    for (int32_t s32_i = 0; s32_i < s32_NumPoints; s32_i++) {
        float32_t f32_Theta = f32_StartAngle + s32_i * (f32_EndAngle - f32_StartAngle) / s32_NumPoints; // // 시작과 끝 각도 사이

        float32_t f32_InterpolatedX = st_CircleCenterPoint.f32_X + f32_Radius * cos(f32_Theta);
        float32_t f32_InterpolatedY = st_CircleCenterPoint.f32_Y + f32_Radius * sin(f32_Theta);

        // 점 추가
        st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedX;
        st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedY;
        st_Planning.st_LocalPath.s32_PathNum++;
    }

    // Point2D_t st_XY1 = {st_Planning.st_GlobalPath.arf32_E[s32_FirstIdx + s32_UturnIdxMargin],
    //                     st_Planning.st_GlobalPath.arf32_N[s32_FirstIdx + s32_UturnIdxMargin]};
    // Point2D_t st_XY2 = {st_Planning.st_GlobalPath.arf32_E[s32_LastIdx - s32_UturnIdxMargin],
    //                     st_Planning.st_GlobalPath.arf32_N[s32_LastIdx - s32_UturnIdxMargin]};

    // float32_t f32_TotalDistance = Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
    //                                           s32_FirstIdx, s32_LastIdx);

    // int32_t s32_PathIter = int32_t(f32_TotalDistance / 0.1); // 0.1m 보간
    // st_Planning.st_LocalPath.s32_PathNum = 0;

    // for (int32_t s32_i = 0; s32_i <= s32_PathIter; s32_i++)
    // {
    //     // 선형 보간 시작
    //     float32_t f32_Num = float32_t(s32_i) / s32_PathIter;
    //     float32_t f32_InterpolatedX = st_XY1.f32_X + f32_Num * (st_XY2.f32_X - st_XY1.f32_X);
    //     float32_t f32_InterpolatedY = st_XY1.f32_Y + f32_Num * (st_XY2.f32_Y - st_XY1.f32_Y);

    //     st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedX;
    //     st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedY;
    //     st_Planning.st_LocalPath.s32_PathNum++;
    // }
}

void Uturn(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Uturn_t &st_Uturn)
{
    s32_FilterObjectCnt = 0;

    float32_t f32_StartIdx_PosDistance = 0, f32_EndIdx_PosDistance = 0;
    Point2D_t st_Direction = {0};
    float32_t f32_Gap = 0;
    std::cout << "--------------------------------------------------------" << std::endl;
    s32_StartIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;
    s32_EndIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second;

    cout << "st_LiDAR.s32_ObjectNum : " << st_LiDAR.s32_ObjectNum << endl;
    if (st_LiDAR.s32_ObjectNum <= 1 && ars32_ObjectIdx[0] == -1)
    {
        // 감지된 Obeject가 없고 지금까지 Uturn 내에서 선택된 ConnectObject가 없다면
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        return;
    }
    if (st_LiDAR.s32_ObjectNum > 1 && !b_UturnPathMakeFlag)
    {
        for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
        {
            if(st_LiDAR.st_XY[s32_i].f32_Y < 0)
            {
                s32_FilterObjectCnt++;
                st_FilterObject[s32_i] = st_LiDAR.st_EN[s32_i];
            }
        }
        if(s32_FilterObjectCnt >= 2)
        {
            // 현재 감지된 Object에 대해서 탐색 Angle 범위 내에 있는 Object 중 나와 거리 제일 먼 Object 탐색
            ConnectObject(st_LiDAR, st_GPSIMU, st_Planning, st_Uturn.f32_AngleThreshold_deg);

            // 제일 먼 Object에 대해서 현재 주행하고 있는 경로와 유턴 후 진행해야 하는 경로 각각의 Near I`dx 선정
            UturnFindObjectNear(st_LiDAR, st_Planning, st_Uturn.s32_Iter, st_Uturn.f32_Gap);
        }

    }
    // std::cout << "s32_FilterObjectCnt : " << s32_FilterObjectCnt << std::endl;
    cout << "s32_FirstIdx : " << s32_FirstIdx << endl;
    cout << "s32_LastIdx : " << s32_LastIdx << endl;
    cout << "st_Planning.s32_GlobalNearIdx : " << st_Planning.s32_GlobalNearIdx << endl;
    cout << "st_Planning.s32_LocalNearIdx : " << st_Planning.s32_LocalNearIdx << endl;
    vector<float32_t> tmp = {st_LastObject.f32_X, st_LastObject.f32_Y};
    VisualizePoint(tmp,Color::ORANGE, 1.5f);
    VisualizeLocalPath(Color::GREEN, 0.5f, st_Planning, st_GPSIMU);
    // cout << "st_Planning.st_LocalPath.s32_PathNum : " << st_Planning.st_LocalPath.s32_PathNum << endl;
    if (s32_FirstIdx - st_Uturn.s32_StartIdxMargin <= st_Planning.s32_GlobalNearIdx &&
        !b_UturnPathMakeFlag)
    {
        // 경로가 만들어 지고, 현재 Near Idx가 위에서 구한 Start Idx + Margin 이후면 만들어진 경로에 대해서 주행
        MakeLinearPath(st_Planning, st_Uturn.s32_IdxMargin);
        // First와 Last 간의 선형 보간을 통해 경로 생성
        // s32_LastIdx -= st_Uturn.s32_IdxMargin;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // int32_t s32_EndIdx = s32_LastIdx + 300;
        int32_t s32_EndIdx = s32_LastIdx + 300;
        MakePath(st_Planning, s32_LastIdx, s32_EndIdx, st_Direction, f32_Gap);

        // 선형 보간 경로(Local) + 전역 경로(Last ~ End) => Local 경로 생성
        b_UturnPathMakeFlag = true;
        // std::cout << "-----------------------------Local" << std::endl;
        // for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i++)
        // {
        //     std::cout << "(" <<st_Planning.st_LocalPath.arf32_E[s32_i] <<"," << st_Planning.st_LocalPath.arf32_N[s32_i] << ")" << std::endl;
        // }
        // // cout << "============================== Uturn Path Make" << endl;
        // std::cout << "Map----------------------------------------" << std::endl;
        // for (int32_t s32_i = s32_FirstIdx - 50; s32_i < s32_LastIdx + 50; s32_i++)
        // {
        //     std::cout << "(" << st_Planning.st_GlobalPath.arf32_E[s32_i] << ", " << st_Planning.st_GlobalPath.arf32_N[s32_i] << ")" << std::endl;
        // }
        // std::cout << "Ego----------------------------------------" << std::endl;
        // std::cout << "(" << st_GPSIMU.f32_E << ", " << st_GPSIMU.f32_N << ")" << std::endl;
    }
    
    else if (b_UturnPathMakeFlag)
    {
        if (st_Planning.s32_LocalNearIdx >= 250)
        {
            if (!b_Local_Global)
            {
                // Local에서 Global로 주행
                st_Planning.s32_GlobalNearIdx = s32_LastIdx;
                b_Local_Global = true;
            }
            cout << "============================== Global Path Tracking" << endl;
            st_Planning.st_Mission.s_SelectName = "TRACKING";
        }
        else
        {
            cout << "============================== Uturn Path Tracking" << endl;
            // 아직 안 도달했으면 Local 주행
            st_Planning.s32_LocalNearIdx = FindNear(st_Planning, st_GPSIMU.f32_E, st_GPSIMU.f32_N, c_FindNearLen + 300, "Local");
            // float32_t f32_Dis = Normalize2D(st_GPSIMU.f32_E,st_GPSIMU.f32_N, st_Planning.st_LocalPath.arf32_E[st_Planning.s32_LocalNearIdx],st_Planning.st_LocalPath.arf32_N[st_Planning.s32_LocalNearIdx]);
            // std::cout << "f32_Dis : " << f32_Dis << std::endl;
            st_Planning.s_PathType = "Local";
        }
    }
    else
    {
        // cout << "============================== Before Uturn" << endl;
        // Local 경로가 안 만들어지고 First Idx + Margin 이전이면 Global 주행
        st_Planning.st_Mission.s_SelectName = "TRACKING";
    }
}