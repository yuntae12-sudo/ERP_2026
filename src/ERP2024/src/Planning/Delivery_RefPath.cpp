#include <Planning/Planning.hpp>

extern int32_t s32_StartIdx, s32_EndIdx;
extern bool b_SlowSignal, b_StaySignal;

extern int32_t ars_DetectNum[3];
extern int32_t s32_DeliveryTargetNum;
extern Point2D_t st_DeliveryFixTarget[1];
extern int32_t ars_DeliveryFixTargetNear[1];
extern std::vector<Point2D_t> vst_DeliverySelectTarget;
extern std::vector<int32_t> vs_DeliverySelectTargetCounting;

extern int32_t s32_DeliverySignal;
// Constol에서 시그널
extern float32_t f32_DeliveryFinishDistance;

void DetectNumber(LiDAR_t &st_LiDAR, Planning_t &st_Planning)
{
    // 배달 A에서만 잡을 수 있도록 + 배달 A 정차 이후에는 보지 않도록
    if (s32_DeliverySignal >= 2 || st_Planning.st_Mission.s_SelectName != "DELIVERY_A")
    {
        return;
    }
    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        switch (st_LiDAR.ars32_Classes[s32_i])
        {
        case 0:
            ars_DetectNum[0] += 1;
            break;
        }
    }
    // s32_DeliveryTargetNum = 101 + std::distance(ars_DetectNum, std::max_element(ars_DetectNum, ars_DetectNum + 3));
    s32_DeliveryTargetNum = 0;
}

void DeliveryTarget(LiDAR_t &st_LiDAR, Planning_t &st_Planning)
{
    // 배달 A or B 정차 이후
    if (s32_DeliverySignal >= 2 || s32_DeliverySignal >= 4)
    {
        return;
    }
    // 감지된 모든 물체에 대해서
    for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
    {
        // 감지된 물체에 대해 모든 것에 counting - upgrade 필요
        bool b_DelveryFlag = false;
        // 제일 많이 감지된 거 == 현재 감지된 거
        if (s32_DeliveryTargetNum == st_LiDAR.ars32_Classes[s32_i])
        {
            // 저장한 target 만큼
            for (int32_t s32_j = 0; s32_j < vst_DeliverySelectTarget.size(); s32_j++)
            {
                // 저장된 거랑 현재 감지된 거랑 비슷하면 갱신 -- upgrade 필요
                // 계속 동일한 방향으로 인식이 잘못되면 결국 이상한 곳으로 갱신됨
                // if (Normalize2D(vst_DeliverySelectTarget[s32_j], st_LiDAR.st_XY[s32_i]) < 0.2)
                // {
                    vst_DeliverySelectTarget[s32_j] = st_LiDAR.st_XY[s32_i];
                    vs_DeliverySelectTargetCounting[s32_j] += 1;
                    b_DelveryFlag = true;
                    break;
                // }
            }
            // 없거나 완전히 이상하게 인식되면 새로 추가 -- upgrade 필요
            if (!b_DelveryFlag)
            {
                vst_DeliverySelectTarget.emplace_back(st_LiDAR.st_XY[s32_i]);
                vs_DeliverySelectTargetCounting.emplace_back(1);
            }
        }
    }
    // 저장된 애들 중에 counting이 제일 많은 애
    int32_t SelectTargetIdx = std::max_element(vs_DeliverySelectTargetCounting.begin(), vs_DeliverySelectTargetCounting.end()) - vs_DeliverySelectTargetCounting.begin();
    st_DeliveryFixTarget[0] = vst_DeliverySelectTarget[SelectTargetIdx];
    // std::cout << "-------------------------------OBJ---------------------------" << std::endl;
    // std::cout << "(" << st_DeliveryFixTarget[0].f32_X << ", " << st_DeliveryFixTarget[0].f32_Y << ")" << std::endl;
    ars_DeliveryFixTargetNear[0] = FindNear_LiDAR(st_Planning, st_DeliveryFixTarget[0], st_Planning.st_RefPath.s32_PathNum);
    // cout << "ars_DeliveryFixTargetNear[0]: " << ars_DeliveryFixTargetNear[0] << endl;
}

void MakeDeliveryPath(LiDAR_t &st_LiDAR, Planning_t &st_Planning, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx, Point2D_t *st_OptimalPoints)
{
    Point2D_t st_DeliveryDirection = {0};
    float32_t f32_DeliveryDist = 0;
    st_Planning.st_LocalPath.s32_PathNum = 0;

    // 시작 near에서 deliv first 까지 넣기
    MakePath_LiDAR(st_Planning, st_Planning.st_RefPath.s32_NearIdx, ps_ObjectFirstLastIdx[0].first, st_DeliveryDirection, f32_DeliveryDist);

    // 가운데 부분
    st_DeliveryDirection = UnitVector(st_Planning.st_RefPath.arf32_X[ars_DeliveryFixTargetNear[0]],
                                   st_Planning.st_RefPath.arf32_Y[ars_DeliveryFixTargetNear[0]],
                                   st_OptimalPoints[0].f32_X, st_OptimalPoints[0].f32_Y);

    f32_DeliveryDist = Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y, ars_DeliveryFixTargetNear[0],
                                st_OptimalPoints[0]);
    if (isnan(st_DeliveryDirection.f32_X) || isnan(st_DeliveryDirection.f32_Y))
    {
        st_DeliveryDirection = {0};
    }
    MakePath_LiDAR(st_Planning, ps_ObjectFirstLastIdx[0].first, ps_ObjectFirstLastIdx[0].second, st_DeliveryDirection, f32_DeliveryDist);
    // 끝 부분
    st_DeliveryDirection = {0};
    f32_DeliveryDist = 0;
    MakePath_LiDAR(st_Planning, ps_ObjectFirstLastIdx[0].second, st_Planning.s32_RefPathEndIdx, st_DeliveryDirection, f32_DeliveryDist);

    // std::cout << "-------------------------------LocalPath---------------------------" << std::endl;
    // for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i++)
    // {
    //     std::cout << "(" << st_Planning.st_LocalPath.arf32_E[s32_i] << ", " << st_Planning.st_LocalPath.arf32_N[s32_i] << ")" << std::endl;
    // }
    // std::cout << "(" << st_Planning.st_LocalPath.arf32_E[0] << ", " << st_Planning.st_LocalPath.arf32_N[0] << ")" << std::endl;
}

void DeliverySignal(Planning_t &st_Planning, int32_t *ars_DeliveryFixTargetNear, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx, int32_t s32_NaN)
{
    // 배달 A에 가까워서 stay 들어갈때
    // cout << "distance: " << Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
    //                                                                         st_Planning.s32_GlobalNearIdx, ars_DeliveryFixTargetNear[0]) << endl;
                                                                            
    // cout << "s32_DeliverySignal : " << s32_DeliverySignal << endl;
    // cout << "st_Planning.b_StaySignal : " << st_Planning.b_StaySignal << endl;
    // cout << "st_Planning.st_Mission.s_SelectName : " << st_Planning.st_Mission.s_SelectName << endl;
    // cout << "ars_DeliveryFixTargetNear[0] : " << ars_DeliveryFixTargetNear[0] << endl;
    float32_t f32_X = 0;
    float32_t f32_Y = 0;

    if (s32_DeliverySignal == 0 && f32_DeliveryFinishDistance > Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y,
                                                                            st_Planning.st_RefPath.s32_NearIdx, ars_DeliveryFixTargetNear[0]))
    {
        st_Planning.b_StaySignal = true;
        s32_DeliverySignal = 1;
    }
    // 배달 A 끝나서 나갈때 초기화
    if (s32_DeliverySignal == 1 && f32_DeliveryFinishDistance < Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y,
                                                                            st_Planning.st_RefPath.s32_NearIdx, ars_DeliveryFixTargetNear[0]))
    {
        ars_DeliveryFixTargetNear[0] = -1;
        ps_ObjectFirstLastIdx[0] = {-1, -1};
        vst_DeliverySelectTarget.clear();
        vs_DeliverySelectTargetCounting.clear();
        st_DeliveryFixTarget[0] = {0};
        s32_DeliveryTargetNum += 10;
        s32_DeliverySignal = 2;
    }
    // 배달 A 끝나서 Stay 원래대로 돌아올때 - 배달 A 완전 끝 정상 주행
    if (s32_DeliverySignal == 2 && st_Planning.b_StaySignal == false)
    {
        s32_DeliverySignal = 3;
        st_Planning.st_Mission.s_SelectName = "TRACKING";
    }
    // 배달 B 가까워서 stay 들어갈때
    if (s32_DeliverySignal == 3 && f32_DeliveryFinishDistance > Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y,
                                                                            st_Planning.st_RefPath.s32_NearIdx, ars_DeliveryFixTargetNear[0]))
    {
        st_Planning.b_StaySignal = true;
        s32_DeliverySignal = 4;
    }
    // 배달 B 끝
    if (s32_DeliverySignal == 4 && f32_DeliveryFinishDistance < Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y,
                                                                            st_Planning.st_RefPath.s32_NearIdx, ars_DeliveryFixTargetNear[0]))
    {
        s32_DeliverySignal = 5;
    }
    if (s32_DeliverySignal == 5 && b_StaySignal == false)
    {
        s32_DeliverySignal = 6;
        st_Planning.st_Mission.s_SelectName = "TRACKING";
    }
}

void Delivery(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Delivery_t &st_Delivery)
{
    s32_StartIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;
    s32_EndIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second;
    std::vector<std::vector<std::vector<float32_t>>> vf_PerpendicularPoints(1);
    std::pair<int32_t, int32_t> ps_ObjectFirstLastIdx[1];
    Point2D_t st_OptimalPoints[1] = {0};

    st_Planning.s32_LocalNearIdx = 0; 
    bool b_etc = false;
    if (st_LiDAR.s32_ObjectNum != 0)
    {
        DetectNumber(st_LiDAR, st_Planning);
        // 처음 deliverynumber 인식 + counting + select
        DeliveryTarget(st_LiDAR, st_Planning);
        // 추가적인 인식을 통한 위치 변화 - 예외처리
    }
    if (ars_DeliveryFixTargetNear[0] == -1)
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        return;
    }

    Perpendicular_LiDAR(st_Planning, ars_DeliveryFixTargetNear, 1,
              st_Delivery.s32_Iter, st_Delivery.f32_Gap, vf_PerpendicularPoints);

    // 선택된 배달 위치에 대해 수직점

    Constraint_LiDAR(st_Planning, st_DeliveryFixTarget, ars_DeliveryFixTargetNear, 1,
                vf_PerpendicularPoints, st_Delivery.f32_Thres, st_Delivery.f32_LeftThres, st_Delivery.f32_RightThres);

    // 수직점들에 대해 제약

    CostFunction_LiDAR(st_GPSIMU, st_Planning, ars_DeliveryFixTargetNear, 1, st_DeliveryFixTarget,
                 vf_PerpendicularPoints, st_OptimalPoints, b_etc, st_Delivery.arf32_Costs);

    // Optimal Point get

    SelectFirstLastPoint_LiDAR(st_Planning, ars_DeliveryFixTargetNear, 1,
                         st_Delivery.f32_FirstLastDist, st_Delivery.f32_FirstLastDist, ps_ObjectFirstLastIdx);

    // 경로 생성할 First, Last 구하기
    MakeDeliveryPath(st_LiDAR, st_Planning, ps_ObjectFirstLastIdx, st_OptimalPoints);
    // 경로 생성
    DeliverySignal(st_Planning, ars_DeliveryFixTargetNear, ps_ObjectFirstLastIdx, 0);
    // 배달 a, b에 대해 시그널을 통해 진입 / 정차 / 나가기
    VisualizeLocalPath_Avoid(Color::RED, 0.5f, st_Planning);
    // VisualizePoint({0.0f, 0.0f}, Color::YELLOW, 1.0f);
}