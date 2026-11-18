#include <Planning/Planning.hpp>
#include <Planning/delaunator.hpp>

using namespace Eigen;
using namespace std;

vector<float64_t> LidarPoints;
vector<vector<float32_t>> TrianglePoints;
vector<vector<float32_t>> LocalPath;
vector<vector<float32_t>> MidPointCandidates;

void SortPoints(vector<vector<float32_t>>& points) {
    sort(points.begin(), points.end(), [](const vector<float32_t>& a, const vector<float32_t>& b) {
        float32_t dist_a = sqrt(pow(a[0], 2) + pow(a[1], 2));
        float32_t dist_b = sqrt(pow(b[0], 2) + pow(b[1], 2));
        return dist_a < dist_b;
    });
}

void LocalPP(vector<float32_t>& Start, vector<float32_t>& End, int32_t NumPoints, vector<vector<float32_t>>& Map) {
    vector<float32_t> Interval;
    for (int32_t s32_i = 0; s32_i < Start.size(); s32_i++) {
        Interval.push_back((End[s32_i] - Start[s32_i]) / (NumPoints - 1));
    }

    for (int32_t s32_i = 0; s32_i < NumPoints; s32_i++) {
        vector<float32_t> point;
        for (int32_t s32_j = 0; s32_j < Start.size(); s32_j++) {
            point.push_back(Start[s32_j] + s32_i * Interval[s32_j]);
        }
        Map.push_back(point);
    }
}

void DelaunayTriangulation(vector<double> &LidarPoints, Planning_t &st_Planning) {
    LocalPath.clear();
    MidPointCandidates.clear();
    TrianglePoints.clear();
    bool b_Only1MidPoint = false;

    // cout << "====================================================" << endl;
    delaunator::Delaunator d(LidarPoints);

    // 직선의 빈도수를 저장하는 맵
    map<vector<vector<float32_t>>, int32_t> TrianglePointsCnt;
    for(int32_t s32_i = 0; s32_i < d.triangles.size(); s32_i += 3) {
        float32_t tx0 = d.coords[2 * d.triangles[s32_i]];
        float32_t ty0 = d.coords[2 * d.triangles[s32_i] + 1];
        float32_t tx1 = d.coords[2 * d.triangles[s32_i + 1]];
        float32_t ty1 = d.coords[2 * d.triangles[s32_i + 1] + 1];
        float32_t tx2 = d.coords[2 * d.triangles[s32_i + 2]];
        float32_t ty2 = d.coords[2 * d.triangles[s32_i + 2] + 1];

        if (isObtuse(tx0, ty0, tx1, ty1, tx2, ty2)) {
            continue; // 둔각 예외 처리
        }

        TrianglePoints.push_back({tx0, ty0});
        TrianglePoints.push_back({tx1, ty1});
        TrianglePoints.push_back({tx2, ty2});

        TrianglePointsCnt[{{tx0, ty0}, {tx1, ty1}}]++;
        TrianglePointsCnt[{{tx1, ty1}, {tx0, ty0}}]++;

        TrianglePointsCnt[{{tx2, ty2}, {tx1, ty1}}]++;
        TrianglePointsCnt[{{tx1, ty1}, {tx2, ty2}}]++;

        TrianglePointsCnt[{{tx0, ty0}, {tx2, ty2}}]++;
        TrianglePointsCnt[{{tx2, ty2}, {tx0, ty0}}]++;

        // printf(
        //     "Triangle points: [[%f, %f], [%f, %f], [%f, %f]]\n",
        //     tx0, ty0,
        //     tx1, ty1,
        //     tx2, ty2
        // );
    }
    removeDuplicates(TrianglePoints); 

    SortPoints(TrianglePoints); // 차와 가까운 순으로 정렬 -> 중점 안나올 때 예외처리용

    for (const auto& Line : TrianglePointsCnt) {
        if (Line.second >= 2) {  // 빈도수가 2 이상인 직선을 찾음
            vector<float32_t> Point1 = Line.first[0]; // 첫 번째 점
            vector<float32_t> Point2 = Line.first[1]; // 두 번째 점

            vector<float32_t> MidPoint = {(Point1[0] + Point2[0]) / 2, (Point1[1] + Point2[1]) / 2};

            if (find(MidPointCandidates.begin(), MidPointCandidates.end(), MidPoint) == MidPointCandidates.end()) {
                MidPointCandidates.push_back(MidPoint);
            }
        }
    }
    // MidPointCandidates 정렬
    SortPoints(MidPointCandidates); // 차와 가까운 순으로 정렬

    if (MidPointCandidates.empty()){
        int32_t LeftCnt = 0, RhtCnt = 0;
        int32_t LeftIdx = -1, RightIdx = -1;

        for (int32_t s32_i = 0; s32_i < TrianglePoints.size(); s32_i++){
            float32_t Angle_tmp = atan2(TrianglePoints[s32_i][1],TrianglePoints[s32_i][0]);
            if (Angle_tmp > 0){
                LeftCnt ++;
                LeftIdx = s32_i;
            }
            else if (Angle_tmp < 0){
                RhtCnt ++;
                RightIdx = s32_i;
            }
        }
        if (LeftCnt == 1){
            cout << "왼쪽에 하나" << endl;
            vector<float32_t> PointRef = {TrianglePoints[LeftIdx][0], TrianglePoints[LeftIdx][1]};
            for (int32_t s32_i = 0; s32_i < TrianglePoints.size(); s32_i++){
                if (s32_i == LeftIdx){
                    continue;
                }
                vector<float32_t> Point_tmp = {TrianglePoints[s32_i][0], TrianglePoints[s32_i][1]};
                vector<float32_t> MidPoint_tmp = {(PointRef[0] + Point_tmp[0]) / 2, (PointRef[1] + Point_tmp[1]) / 2};
                MidPointCandidates.push_back(MidPoint_tmp);
            }
        }
        if (RhtCnt == 1){
            cout << "오른쪽에 하나" << endl;
            vector<float32_t> PointRef = {TrianglePoints[RightIdx][0], TrianglePoints[RightIdx][1]};
            for (int32_t s32_i = 0; s32_i < TrianglePoints.size(); s32_i++){
                if (s32_i == RightIdx){
                    continue;
                }
                vector<float32_t> Point_tmp = {TrianglePoints[s32_i][0], TrianglePoints[s32_i][1]};
                vector<float32_t> MidPoint_tmp = {(PointRef[0] + Point_tmp[0]) / 2, (PointRef[1] + Point_tmp[1]) / 2};
                MidPointCandidates.push_back(MidPoint_tmp);
            }
        }
    }
    else if (MidPointCandidates.size() == 1){
        cout << "중점 하나, 차량과 중점 Local Path" << endl; 
        MidPointCandidates.insert(MidPointCandidates.begin(), {0, 0});
        b_Only1MidPoint = true;
    }
    
    // 처음 RefAngle은 Ego와 MidPoint_Candidates중 가장 첫번째 점
    float32_t f32_AngleThres = 35.0 * M_PI / 180.0; // TODO : ㄴ추후 여기 파라미터 수정봐야함

    float32_t f32_RefAngle = atan2(MidPointCandidates[0][1], MidPointCandidates[0][0]);

    if(f32_RefAngle < -f32_AngleThres || f32_RefAngle > f32_AngleThres) {
        MidPointCandidates.erase(MidPointCandidates.begin()); // 첫번째 점이 라바콘 경계에 위치해있을 때, 예외처리
    }

    for (int32_t s32_i = 0; s32_i < MidPointCandidates.size() - 1;) {
        if (b_Only1MidPoint) break;
        
        float32_t f32_MaxAngle = f32_RefAngle + f32_AngleThres;
        float32_t f32_MinAngle = f32_RefAngle - f32_AngleThres;
        cout << "f32_MinAngle : " << f32_MinAngle << ", " << "f32_MaxAngle : " << f32_MaxAngle << endl;
        // cout << " i " << i << endl;
        float32_t f32_RefAngle_tmp = atan2(MidPointCandidates[s32_i + 1][1] - MidPointCandidates[s32_i][1],
                                       MidPointCandidates[s32_i + 1][0] - MidPointCandidates[s32_i][0]);
        
        if (f32_RefAngle_tmp < f32_MinAngle || f32_RefAngle_tmp > f32_MaxAngle) {
            cout << "예외처리 할 f32_RefAngle_tmp : " << f32_RefAngle_tmp << endl;
            MidPointCandidates.erase(MidPointCandidates.begin() + (s32_i + 1));
        } else {
            cout << "PASS할 f32_RefAngle_tmp : " << f32_RefAngle_tmp << endl;
            f32_RefAngle = f32_RefAngle_tmp;
            s32_i++;
        }
    }
    // cout << "MidPointCandidates.size() : " << MidPointCandidates.size() << endl;
    
    for (int32_t s32_i = 0; s32_i < MidPointCandidates.size() - 1; s32_i++) {
        vector<float32_t> Start = {static_cast<float32_t>(MidPointCandidates[s32_i][0]), static_cast<float32_t>(MidPointCandidates[s32_i][1])};
        vector<float32_t> End = {static_cast<float32_t>(MidPointCandidates[s32_i + 1][0]), static_cast<float32_t>(MidPointCandidates[s32_i + 1][1])};
        LocalPP(Start, End, 15, LocalPath);
    }

    for (int32_t s32_i = 0; s32_i < LocalPath.size(); s32_i++) {
        st_Planning.st_LocalPath.arf32_E[s32_i] = LocalPath[s32_i][0];
        st_Planning.st_LocalPath.arf32_N[s32_i] = LocalPath[s32_i][1];
    }
    st_Planning.st_LocalPath.s32_PathNum = LocalPath.size();

    cout << "LocalPath.size() : " << LocalPath.size() << endl;
    cout << "==============================" << endl;
}

void Narrow(LiDAR_t &st_LiDAR, Planning_t &st_Planning)
{
    if (st_LiDAR.s32_ObjectNum != 0)
    {
        LidarPoints.clear();
        LidarPoints.resize(0);
        for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
        {
            LidarPoints.push_back(st_LiDAR.st_XY[s32_i].f32_X);
            LidarPoints.push_back(st_LiDAR.st_XY[s32_i].f32_Y);
        }
        DelaunayTriangulation(LidarPoints, st_Planning);
    }
}
