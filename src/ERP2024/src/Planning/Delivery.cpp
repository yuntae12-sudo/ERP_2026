#include <Planning/Planning.hpp>

extern int32_t s32_StartIdx, s32_EndIdx;
extern bool b_SlowSignal;

int32_t ars_DetectNum[3] = {0};
int32_t s32_DeliveryTargetNum = 0;
Point2D_t st_DeliveryFixTarget[4] = {0};
int32_t ars_DeliveryFixTargetNear[4] = {-1};
std::vector<Point2D_t> vst_DeliverySelectTarget;
std::vector<int32_t> vs_DeliverySelectTargetCounting;

int32_t s32_DeliveryBStartIdxMargin = 30;
int32_t s32_DeliverySignal = 0;
// Constol에서 시그널
// float32_t f32_DeliveryFinishDistance = 2.5;
float32_t f32_DeliveryFinishDistance = 1; // BBBBB
int32_t s32_StopPointNearIdx = -1;
bool tmp = false;

void SortDeliveryFixTargets(GPSIMU_t &st_GPSIMU) {
    std::sort(st_DeliveryFixTarget, st_DeliveryFixTarget + 4,
              [&st_GPSIMU](Point2D_t &A, Point2D_t &B) { 
                  return Normalize2D(A.f32_X, A.f32_Y, st_GPSIMU.f32_E, st_GPSIMU.f32_N) 
                       < Normalize2D(B.f32_X, B.f32_Y, st_GPSIMU.f32_E, st_GPSIMU.f32_N);
              });
}

void DetectNumber(Fusion_t &st_Fusion, Planning_t &st_Planning)
{
    // 배달 A에서만 잡을 수 있도록 + 배달 A 정차 이후에는 보지 않도록
    if (s32_DeliverySignal >= 2 || st_Planning.st_Mission.s_SelectName != "DELIVERY_A")
    {
        return;
    }
    for (int32_t s32_i = 0; s32_i < st_Fusion.s32_ObjectNum; s32_i++)
    {
        switch (st_Fusion.ars32_Classes[s32_i])
        {
        case 101:
            ars_DetectNum[0] += 1;
            break;
        case 102:
            ars_DetectNum[1] += 1;
            break;
        case 103:
            ars_DetectNum[2] += 1;
            break;
        }
    }
    s32_DeliveryTargetNum = 101 + std::distance(ars_DetectNum, std::max_element(ars_DetectNum, ars_DetectNum + 3));
}

void DeliveryTarget(Fusion_t &st_Fusion, Planning_t &st_Planning)
{
    // 배달 A or B 정차 이후
    // if (s32_DeliverySignal >= 2 || s32_DeliverySignal >= 4)
    // {
        // return;
    // }
    // 배달 A
    if(s32_DeliverySignal < 3)
    {
        // 감지된 모든 물체에 대해서
        for (int32_t s32_i = 0; s32_i < st_Fusion.s32_ObjectNum; s32_i++)
        {
            // 감지된 물체에 대해 모든 것에 counting - upgrade 필요
            bool b_DelveryFlag = false;
            // 제일 많이 감지된 거 == 현재 감지된 거
            if (s32_DeliveryTargetNum == st_Fusion.ars32_Classes[s32_i])
            {
                // 저장한 target 만큼
                for (int32_t s32_j = 0; s32_j < vst_DeliverySelectTarget.size(); s32_j++)
                {
                    // 저장된 거랑 현재 감지된 거랑 비슷하면 갱신 -- upgrade 필요
                    // 계속 동일한 방향으로 인식이 잘못되면 결국 이상한 곳으로 갱신됨
                    if (Normalize2D(vst_DeliverySelectTarget[s32_j], st_Fusion.st_EN[s32_i]) < 0.2)
                    {
                        vst_DeliverySelectTarget[s32_j] = st_Fusion.st_EN[s32_i];
                        vs_DeliverySelectTargetCounting[s32_j] += 1;
                        b_DelveryFlag = true;
                        break;
                    }
                }
                // 없거나 완전히 이상하게 인식되면 새로 추가 -- upgrade 필요
                if (!b_DelveryFlag)
                {
                    vst_DeliverySelectTarget.emplace_back(st_Fusion.st_EN[s32_i]);
                    vs_DeliverySelectTargetCounting.emplace_back(1);
                }
            }
        }
    
        int32_t SelectTargetIdx = std::max_element(vs_DeliverySelectTargetCounting.begin(), vs_DeliverySelectTargetCounting.end()) - vs_DeliverySelectTargetCounting.begin();
        // 저장된 애들 중에 counting이 제일 많은 애
        st_DeliveryFixTarget[0] = vst_DeliverySelectTarget[SelectTargetIdx];
        ars_DeliveryFixTargetNear[0] = FindNear(st_Planning, st_DeliveryFixTarget[0], s32_StartIdx, 800, "Global");
    }
    // 배달 B
    else
    {
        for (int32_t s32_i = 0; s32_i < st_Fusion.s32_ObjectNum; s32_i++)
        {
            if (st_Fusion.ars32_Classes[s32_i] == 111) // B1
            {
                st_DeliveryFixTarget[1] = st_Fusion.st_EN[s32_i];
                ars_DeliveryFixTargetNear[1] = FindNear(st_Planning, st_DeliveryFixTarget[1], s32_StartIdx, 800, "Global");

                Point2D_t st_UnitVector_tmp = UnitVector(st_Planning.st_GlobalPath.arf32_E[ars_DeliveryFixTargetNear[1]],st_Planning.st_GlobalPath.arf32_N[ars_DeliveryFixTargetNear[1]],
                                                         st_Planning.st_GlobalPath.arf32_E[ars_DeliveryFixTargetNear[1] - 1],st_Planning.st_GlobalPath.arf32_N[ars_DeliveryFixTargetNear[1] - 1]);

                float32_t f32_GapDistance = 5.0;

                st_DeliveryFixTarget[0] = {st_DeliveryFixTarget[1].f32_X + f32_GapDistance*st_UnitVector_tmp.f32_X,
                                           st_DeliveryFixTarget[1].f32_Y + f32_GapDistance*st_UnitVector_tmp.f32_Y};
                ars_DeliveryFixTargetNear[0] = FindNear(st_Planning, st_DeliveryFixTarget[0], s32_StartIdx, 800, "Global");
            }
            if (st_Fusion.ars32_Classes[s32_i] == 112) // B2
            {
                st_DeliveryFixTarget[2] = st_Fusion.st_EN[s32_i];
                ars_DeliveryFixTargetNear[2] = FindNear(st_Planning, st_DeliveryFixTarget[2], s32_StartIdx, 800, "Global");
            }
            if (st_Fusion.ars32_Classes[s32_i] == 113) // B3
            {
                st_DeliveryFixTarget[3] = st_Fusion.st_EN[s32_i];
                ars_DeliveryFixTargetNear[3] = FindNear(st_Planning, st_DeliveryFixTarget[3], s32_StartIdx, 800, "Global");
            }
        }
    }
}

void MakeDeliveryPath(Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx, Point2D_t *st_OptimalPoints)
{
    if(s32_DeliverySignal < 3)
    {
        Point2D_t st_DeliveryDirection = {0};
        float32_t f32_DeliveryDist = 0;
        st_Planning.st_LocalPath.s32_PathNum = 0;

        // 시작 near에서 deliv first 까지 넣기
        MakePath(st_Planning, st_Planning.s32_GlobalNearIdx, ps_ObjectFirstLastIdx[0].first, st_DeliveryDirection, f32_DeliveryDist);
        // 가운데 부분
        st_DeliveryDirection = UnitVector(st_Planning.st_GlobalPath.arf32_E[ars_DeliveryFixTargetNear[0]],
                                        st_Planning.st_GlobalPath.arf32_N[ars_DeliveryFixTargetNear[0]],
                                        st_OptimalPoints[0].f32_X, st_OptimalPoints[0].f32_Y);

        f32_DeliveryDist = Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
                                    ars_DeliveryFixTargetNear[0], st_OptimalPoints[0]);
        if (isnan(st_DeliveryDirection.f32_X) || isnan(st_DeliveryDirection.f32_Y))
        {
            st_DeliveryDirection = {0};
        }
        MakePath(st_Planning, ps_ObjectFirstLastIdx[0].first, ps_ObjectFirstLastIdx[0].second, st_DeliveryDirection, f32_DeliveryDist);
        // 끝 부분
        st_DeliveryDirection = {0};
        f32_DeliveryDist = 0;
        MakePath(st_Planning, ps_ObjectFirstLastIdx[0].second, s32_EndIdx, st_DeliveryDirection, f32_DeliveryDist);

        std::cout << "-------------------------------LocalPath---------------------------" << std::endl;
        for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i++)
        {
            std::cout << "(" << st_Planning.st_LocalPath.arf32_E[s32_i] << ", " << st_Planning.st_LocalPath.arf32_N[s32_i] << ")" << std::endl;
        }
    }
    else // 배달 B
    {
        //배달 B 점들에 대해 거리순 정렬
        SortDeliveryFixTargets(st_GPSIMU);
        //배달 B NearPoint 거리순 정렬
        sort(begin(ars_DeliveryFixTargetNear), end(ars_DeliveryFixTargetNear));

        Point2D_t st_XY1 = {st_DeliveryFixTarget[s32_DeliveryTargetNum - 111].f32_X,
                            st_DeliveryFixTarget[s32_DeliveryTargetNum - 111].f32_Y};
        Point2D_t st_XY2 = {0,0};
        for (int32_t s32_i = 0; s32_i < 4; s32_i++)
        {
            if(st_DeliveryFixTarget[s32_i].f32_X == st_DeliveryFixTarget[s32_DeliveryTargetNum - 111].f32_X)
            {
                st_XY2 = {st_DeliveryFixTarget[s32_i - 1].f32_X,
                          st_DeliveryFixTarget[s32_i - 1].f32_Y};
            }
        }
        float32_t f32_TotalDistance = Normalize2D(st_XY1, st_XY2);

        int32_t s32_PathIter = int32_t(f32_TotalDistance / 0.1); // 0.1m 보간
        st_Planning.st_LocalPath.s32_PathNum = 0;

        for (int32_t s32_i = 0; s32_i <= s32_PathIter; s32_i++)
        {
            // 선형 보간 시작
            float32_t f32_Num = float32_t(s32_i) / s32_PathIter;
            float32_t f32_InterpolatedX = st_XY1.f32_X + f32_Num * (st_XY2.f32_X - st_XY1.f32_X);
            float32_t f32_InterpolatedY = st_XY1.f32_Y + f32_Num * (st_XY2.f32_Y - st_XY1.f32_Y);

            st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedX;
            st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum] = f32_InterpolatedY;
            st_Planning.st_LocalPath.s32_PathNum++;
        }
        
        cout << s32_DeliveryTargetNum << " 배달 포인트에 대해 Local Path 생성 및 진입" << endl;

        Point2D_t st_StopPoint = {(st_XY1.f32_X + st_XY2.f32_X) / 2,
                                  (st_XY1.f32_Y + st_XY2.f32_Y) / 2};
        s32_StopPointNearIdx = FindNear(st_Planning, st_StopPoint, s32_StartIdx, 800, "Global"); 
    }
}

void DeliverySignal(Planning_t &st_Planning, int32_t *ars_DeliveryFixTargetNear, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx)
{
    // 배달 A에 가까워서 stay 들어갈때
    if (s32_DeliverySignal == 0 && f32_DeliveryFinishDistance > Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
                                                                            st_Planning.s32_GlobalNearIdx, ars_DeliveryFixTargetNear[0]))
    {
        st_Planning.b_StaySignal = true;
        s32_DeliverySignal = 1;
    }
    // 배달 A 끝나서 나갈때 초기화
    if (s32_DeliverySignal == 1 && f32_DeliveryFinishDistance < Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
                                                                            st_Planning.s32_GlobalNearIdx, ars_DeliveryFixTargetNear[0]))
    {
        ars_DeliveryFixTargetNear[0] = -1;
        ps_ObjectFirstLastIdx[0] = {-1, -1};
        vst_DeliverySelectTarget.clear();
        vs_DeliverySelectTargetCounting.clear();
        st_DeliveryFixTarget[0] = {0};
        s32_DeliveryTargetNum += 10;
        s32_DeliverySignal = 3;
        st_Planning.b_SlowSignal = false;
    }
    // 배달 A 끝나서 Stay 원래대로 돌아올때 - 배달 A 완전 끝 정상 주행
    if (s32_DeliverySignal == 2 && st_Planning.b_StaySignal == false)
    {
        s32_DeliverySignal = 3;
        st_Planning.st_Mission.s_SelectName = "TRACKING";
    }
    // 배달 B 가까워서 stay 들어갈때
    cout << "Normalize2D(StopIdx && EGo)" << Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
                                                                            st_Planning.s32_GlobalNearIdx, ars_DeliveryFixTargetNear[0]) << endl;

    if (s32_DeliverySignal == 3 && f32_DeliveryFinishDistance > Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
                                                                            st_Planning.s32_GlobalNearIdx, s32_StopPointNearIdx))
    {
        st_Planning.b_StaySignal = true;
        // s32_DeliverySignal = 4;
        ars_DeliveryFixTargetNear[s32_DeliveryTargetNum - 111] = -1;
        ps_ObjectFirstLastIdx[0] = {-1, -1};
        vst_DeliverySelectTarget.clear();
        vs_DeliverySelectTargetCounting.clear();
        st_DeliveryFixTarget[0] = {0};
        s32_DeliverySignal = 5;
        st_Planning.b_SlowSignal = false;
    }
    // 배달 B 끝
    if (s32_DeliverySignal == 4 && f32_DeliveryFinishDistance < Normalize2D(st_Planning.st_GlobalPath.arf32_E, st_Planning.st_GlobalPath.arf32_N,
                                                                            st_Planning.s32_GlobalNearIdx, s32_StopPointNearIdx))
    {
        st_Planning.b_SlowSignal = false;
        s32_DeliverySignal = 4;
    }
    if (s32_DeliverySignal == 5 && st_Planning.b_StaySignal == false)
    {
        s32_DeliverySignal = 6;
        st_Planning.st_Mission.s_SelectName = "TRACKING";
    }
}

void Delivery(Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Delivery_t &st_Delivery)
{
    s32_StartIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].first;
    s32_EndIdx = st_Planning.st_Mission.arp_Idx[st_Planning.st_Mission.s32_SelectNumer].second;
    std::vector<std::vector<std::vector<float32_t>>> vf_PerpendicularPoints(1);
    std::pair<int32_t, int32_t> ps_ObjectFirstLastIdx[1];
    Point2D_t st_OptimalPoints[1] = {0};
    // if(!tmp)
    // {
        // s32_DeliverySignal = 3;
        // tmp = true;
        // s32_DeliveryTargetNum = 113;
    // }
    st_Planning.s32_LocalNearIdx = 0;
    bool b_etc = false;
    if(s32_DeliverySignal == 6)
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        return;
    }
    if (st_Fusion.s32_ObjectNum != 0)
    {
        st_Planning.b_SlowSignal = true;
        DetectNumber(st_Fusion, st_Planning);
        // 처음 deliverynumber 인식 + counting + select

        DeliveryTarget(st_Fusion, st_Planning);
        // 추가적인 인식을 통한 위치 변화 - 예외처리
    }
    if (ars_DeliveryFixTargetNear[0] == -1)
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        return;
    }
    cout << "s32_DeliverySignal : " << s32_DeliverySignal<< endl;
    cout << "ars_DeliveryFixTargetNear[0] : " << ars_DeliveryFixTargetNear[0]<< endl;
    cout << "ars_DeliveryFixTargetNear[1] : " << ars_DeliveryFixTargetNear[1]<< endl;
    cout << "ars_DeliveryFixTargetNear[2] : " << ars_DeliveryFixTargetNear[2]<< endl;
    cout << "ars_DeliveryFixTargetNear[3] : " << ars_DeliveryFixTargetNear[3]<< endl;
    cout << "st_Planning.s32_GlobalNearIdx : " << st_Planning.s32_GlobalNearIdx << endl;
    cout << "s32_DeliveryTargetNum - 111 : " << s32_DeliveryTargetNum - 111 << std::endl;
    cout << "ars_DeliveryFixTargetNear[s32_DeliveryTargetNum - 111] : " << ars_DeliveryFixTargetNear[s32_DeliveryTargetNum - 111]<< std::endl;
    cout << "=========================================================" << endl;
    
    
    if (s32_DeliverySignal < 3)
    {
        Perpendicular(st_Planning, ars_DeliveryFixTargetNear, 1,
                    st_Delivery.s32_Iter, st_Delivery.f32_Gap, vf_PerpendicularPoints, "Global");
        // 선택된 배달 위치에 대해 수직점

        Constraint(st_Planning, st_DeliveryFixTarget, 1, vf_PerpendicularPoints, st_Delivery.f32_Thres);
        // 수직점들에 대해 제약

        CostFunction(st_GPSIMU, st_Planning, ars_DeliveryFixTargetNear, 1, st_DeliveryFixTarget,
                    vf_PerpendicularPoints, st_OptimalPoints, b_etc, st_Delivery.arf32_Costs);
        // Optimal Point get

        SelectFirstLastPoint(st_Planning, ars_DeliveryFixTargetNear, 1,
                            st_Delivery.f32_FirstLastDist, st_Delivery.f32_FirstLastDist, ps_ObjectFirstLastIdx);
        // 경로 생성할 First, Last 구하기
    }
    else if (3 < s32_DeliverySignal)
    {
        if (ars_DeliveryFixTargetNear[s32_DeliveryTargetNum - 111] == 0 
            || st_Planning.s32_GlobalNearIdx < ars_DeliveryFixTargetNear[s32_DeliveryTargetNum - 111] - s32_DeliveryBStartIdxMargin)
        {
            st_Planning.st_Mission.s_SelectName = "TRACKING";
            cout << "아직 진입 대기 Global 추종" << endl;
            return;
        }
    }


    MakeDeliveryPath(st_Fusion, st_GPSIMU, st_Planning, ps_ObjectFirstLastIdx, st_OptimalPoints);

    DeliverySignal(st_Planning, ars_DeliveryFixTargetNear, ps_ObjectFirstLastIdx);
    // 배달 a, b에 대해 시그널을 통해 진입 / 정차 / 나가기

    VisualizeLocalPath(Color::GREEN, 0.5f, st_Planning, st_GPSIMU);
}