#include <Planning/Planning.hpp>ㅑ뱌

// MORAI
// #define ERP_MODE 0
// #define PATH_OFFSET 1.4f
// #define ROAD_WIDTH 5.0f
// #define GLOBAL_PATH_WIDTH 3.0f
// #define BAND_WIDTH 1.5f
// #define MORAI_ROI_OFFSET 1.0f
// #define ENTRANCE_GAP 3.5f
// #define BUFFER_SIZE 50
// #define STRAIGHT_STOP_DIST 2.6f
// #define DIAGONAL_PATH_DIST 2.5f
// #define DIAGONAL_STOP_DIST 3.2f
// #define GEARMODE_R_CHANGE_TIME 2  // [s]
// #define GEARMODE_D_CHANGE_TIME 5  // [s]
// #define PARKING_IN_STOP_DIST 3.2f
// #define KEYPOINT_PARALLEL_PATH_DIST 0.5f
// #define ESCAPE_DIST 2.5f

// Siheung
#define ERP_MODE 0 // DONE
#define PATH_OFFSET 0.75f // DONE
#define ROAD_WIDTH 5.0f // DONE
#define GLOBAL_PATH_WIDTH 3.0f // DONE
#define BAND_WIDTH 1.5f // DONE
#define MORAI_ROI_OFFSET 0.9f // DONE
#define ENTRANCE_GAP 3.5f // DONE
#define BUFFER_SIZE 50 // DONE
#define STRAIGHT_STOP_DIST 2.8f // DONE
#define DIAGONAL_PATH_DIST 2.5f // DONE
#define DIAGONAL_STOP_DIST 2.0f // DONE
#define GEARMODE_R_CHANGE_TIME 2  // [s] DONE
#define GEARMODE_D_CHANGE_TIME 5  // [s] DONE
#define PARKING_IN_STOP_DIST 1.7f // DONE not too small not too big!!!
#define KEYPOINT_PARALLEL_PATH_DIST 0.0f // DONE
#define ESCAPE_DIST 2.5f

int32_t s32_IsMapLoaded = 0;
int32_t s32_GlobalStartIdx, s32_GlobalEndIdx;
vector<float32_t> vf32_GlobalStart, vf32_GlobalEnd;  // {E, N}
float32_t f32_GlobalIncline, f32_GlobalIntercept, f32_GlobalDirection_rad;

vector<vector<float32_t>> vf32_ObsInfo;  // {X, Y, E, N}

int32_t s32_EntranceCheckBuffer = 0;

vector<vector<float32_t>> vf32_KeyPointStack;  // {E, N}
vector<float32_t> vf32_KeyPoint;  // {E, N}

int32_t s32_GearmodeChangeCnt = 0;

int32_t s32_IsParkingEnded = 0;

int32_t s32_IsLaneCrossed = 0;

/**************************************************/

namespace Sequence
{
    const int32_t SCAN_AREA = 0;
    const int32_t SAVE_KEY_POINT = 1;
    const int32_t GO_STRAIGHT = 2;
    const int32_t SET_DIAGONAL = 3;
    const int32_t CHANGE_GEARMODE_R = 4;
    const int32_t GO_BACKWARD = 5;
    const int32_t PARALLEL_PARKING = 6;
    const int32_t CHANGE_GEARMODE_D = 7;
    const int32_t ESCAPE = 8;
}

int32_t s32_Sequence = Sequence::SCAN_AREA;

/**************************************************/

float32_t GetLaneDist(float32_t f32_X,
					  float32_t f32_Y,
					  float32_t &f32_LaneIncline,
					  float32_t &f32_LaneIntercept)
{
	return abs(f32_LaneIncline * f32_X - f32_Y + f32_LaneIntercept) / sqrt(pow(f32_LaneIncline, 2) + 1.0f);
}

float32_t GetGlobalDist(float32_t f32_E,
                        float32_t f32_N)
{
	return abs(f32_GlobalIncline * f32_E - f32_N + f32_GlobalIntercept) / sqrt(pow(f32_GlobalIncline, 2) + 1.0f);
}

int32_t FilterParkingLane(float32_t &f32_LaneIncline,
			              float32_t &f32_LaneIntercept,
                          float32_t &f32_LaneHeading_rad,
			              LiDAR_t &st_LiDAR)
{
    if (99.999f < st_LiDAR.arf32_MiddleLaneYIntercept && st_LiDAR.arf32_MiddleLaneYIntercept < 100.001f)
	{
		return 1;
	}

	if (st_LiDAR.arf32_MiddleLaneYIntercept < 0.0f)
	{
		return 1;
	}

	f32_LaneIncline = st_LiDAR.arf32_MiddleLaneSlope;
	f32_LaneIntercept = st_LiDAR.arf32_MiddleLaneYIntercept;
    f32_LaneHeading_rad = -1.0f * atan(f32_LaneIncline);

	return 0;
}

void FilterParkingObs(float32_t &f32_LaneIncline,
                      float32_t &f32_LaneIntercept,
			          LiDAR_t &st_LiDAR)
{
    if (st_LiDAR.s32_ObjectNum > 0)
	{
        if(ERP_MODE)
        {
            float32_t f32_Obs_LaneDist;

            for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
            {
                f32_Obs_LaneDist = GetLaneDist(st_LiDAR.st_XY[s32_i].f32_X, st_LiDAR.st_XY[s32_i].f32_Y, f32_LaneIncline, f32_LaneIntercept);
                
                if (ROAD_WIDTH - BAND_WIDTH / 2.0f < f32_Obs_LaneDist && f32_Obs_LaneDist < ROAD_WIDTH + BAND_WIDTH / 2.0f)
                {
                    vf32_ObsInfo.push_back({st_LiDAR.st_XY[s32_i].f32_X, st_LiDAR.st_XY[s32_i].f32_Y, st_LiDAR.st_EN[s32_i].f32_X, st_LiDAR.st_EN[s32_i].f32_Y});
                }
            }
        }
        else
        {
            for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
            {
                if (-MORAI_ROI_OFFSET - BAND_WIDTH / 2.0f < st_LiDAR.st_XY[s32_i].f32_Y && st_LiDAR.st_XY[s32_i].f32_Y < -MORAI_ROI_OFFSET + BAND_WIDTH / 2.0f)
                {
                    vf32_ObsInfo.push_back({st_LiDAR.st_XY[s32_i].f32_X, st_LiDAR.st_XY[s32_i].f32_Y, st_LiDAR.st_EN[s32_i].f32_X, st_LiDAR.st_EN[s32_i].f32_Y});
                }
            }
        }

		if (vf32_ObsInfo.size() > 0)
		{
			sort(vf32_ObsInfo.begin(), vf32_ObsInfo.end(), [](const vector<float32_t> &a, const vector<float32_t> &b)
			{ return a[0] < b[0]; });
		}
	}
}

void FilterKeyPoint(int32_t &s32_KeyPointIdx)
{
    s32_KeyPointIdx = -1;

    if (vf32_ObsInfo.size() > 1)
	{
        for (int32_t s32_i = 0; s32_i < vf32_ObsInfo.size() - 1; s32_i++)
		{
            if (Normalize2D(vf32_ObsInfo[s32_i][0], vf32_ObsInfo[s32_i][1], vf32_ObsInfo[s32_i + 1][0], vf32_ObsInfo[s32_i + 1][1]) > ENTRANCE_GAP)
            {
                s32_KeyPointIdx = s32_i + 1;
                break;
            }
        }
    }
}

void OffsetPathGen(float32_t f32_OffsetDist,
			       Planning_t &st_Planning)
{
	int32_t s32_PathNum = 0;

    for (int32_t s32_i = s32_GlobalStartIdx; s32_i < s32_GlobalEndIdx; s32_i++)
    {
        st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Planning.st_GlobalPath.arf32_E[s32_i] + sin(f32_GlobalDirection_rad + M_PI / 2.0f) * f32_OffsetDist;
        st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Planning.st_GlobalPath.arf32_N[s32_i] + cos(f32_GlobalDirection_rad + M_PI / 2.0f) * f32_OffsetDist;
		s32_PathNum++;
    }

    st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;
}

void DiagonalOutPathGen(float32_t f32_DiagonalPathDist,
                        Planning_t &st_Planning)
{
    int32_t s32_PathNum = 0;

    float32_t f32_StandardE = vf32_KeyPoint[0] + sin(f32_GlobalDirection_rad + M_PI) * f32_DiagonalPathDist;
    float32_t f32_StandardN = vf32_KeyPoint[1] + cos(f32_GlobalDirection_rad + M_PI) * f32_DiagonalPathDist;

    for (int32_t s32_i = 0; s32_i < 100; s32_i++)
    {
        st_Planning.st_LocalPath.arf32_E[s32_PathNum] = f32_StandardE - sin(f32_GlobalDirection_rad + M_PI / 4.0f * 3.0f) * s32_i * 0.1f;
        st_Planning.st_LocalPath.arf32_N[s32_PathNum] = f32_StandardN - cos(f32_GlobalDirection_rad + M_PI / 4.0f * 3.0f) * s32_i * 0.1f;
		s32_PathNum++;
    }

    st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;
}

void DiagonalInPathGen(float32_t f32_DiagonalPathDist,
                       Planning_t &st_Planning)
{
    int32_t s32_PathNum = 0;

    float32_t f32_StandardE = vf32_KeyPoint[0] + sin(f32_GlobalDirection_rad + M_PI) * f32_DiagonalPathDist;
    float32_t f32_StandardN = vf32_KeyPoint[1] + cos(f32_GlobalDirection_rad + M_PI) * f32_DiagonalPathDist;

    for (int32_t s32_i = 100; s32_i < -100; s32_i--)
    {
        st_Planning.st_LocalPath.arf32_E[s32_PathNum] = f32_StandardE - sin(f32_GlobalDirection_rad + M_PI / 4.0f * 3.0f) * s32_i * 0.1f;
        st_Planning.st_LocalPath.arf32_N[s32_PathNum] = f32_StandardN - cos(f32_GlobalDirection_rad + M_PI / 4.0f * 3.0f) * s32_i * 0.1f;
		s32_PathNum++;
    }

    st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;
}

void ParallelPathGen(float32_t f32_KeyPoint_ParallelDist,
			         Planning_t &st_Planning)
{
	int32_t s32_PathNum = 0;

    st_Planning.st_LocalPath.arf32_E[s32_PathNum] = vf32_KeyPoint[0] + sin(f32_GlobalDirection_rad + M_PI / 2.0f) * f32_KeyPoint_ParallelDist;
    st_Planning.st_LocalPath.arf32_N[s32_PathNum] = vf32_KeyPoint[1] + cos(f32_GlobalDirection_rad + M_PI / 2.0f) * f32_KeyPoint_ParallelDist;
    s32_PathNum++;

    for (int32_t s32_i = 0; s32_i < 100; s32_i++)
    {
        st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Planning.st_LocalPath.arf32_E[s32_PathNum - 1] - sin(f32_GlobalDirection_rad) * s32_i * 0.1f;
        st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Planning.st_LocalPath.arf32_N[s32_PathNum - 1] - cos(f32_GlobalDirection_rad) * s32_i * 0.1f;
		s32_PathNum++;
    }

    st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;
}

void EscapePathGen(GPSIMU_t &st_GPSIMU,
                   Planning_t &st_Planning)
{
    int32_t s32_PathNum = 0;

    float32_t f32_StandardE = st_GPSIMU.f32_E;
    float32_t f32_StandardN = st_GPSIMU.f32_N;

    for (int32_t s32_i = 0; s32_i < 100; s32_i++)
    {
        st_Planning.st_LocalPath.arf32_E[s32_PathNum] = f32_StandardE - sin(f32_GlobalDirection_rad + M_PI / 3.0f * 2.0f) * s32_i * 0.1f;
        st_Planning.st_LocalPath.arf32_N[s32_PathNum] = f32_StandardN - cos(f32_GlobalDirection_rad + M_PI / 3.0f * 2.0f) * s32_i * 0.1f;
		s32_PathNum++;
    }

    st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;
}

/**************************************************/

void ScanArea(float32_t &f32_LaneIncline,
              float32_t &f32_LaneIntercept,
              float32_t &f32_LaneHeading_rad,
              int32_t &s32_IsLaneFiltered,
              int32_t &s32_KeyPointIdx,
              LiDAR_t &st_LiDAR,
              GPSIMU_t &st_GPSIMU,
              Planning_t &st_Planning)
{
    cout << "SEQ: SCAN_AREA" << endl;

    if (st_Planning.st_LocalPath.s32_PathNum == 0)
    {
        OffsetPathGen(PATH_OFFSET, st_Planning);
    }

    s32_IsLaneFiltered = FilterParkingLane(f32_LaneIncline,
                                           f32_LaneIntercept,
                                           f32_LaneHeading_rad,
                                           st_LiDAR);

    if (s32_IsLaneFiltered == 0)
	{
        FilterParkingObs(f32_LaneIncline,
                         f32_LaneIntercept,
			             st_LiDAR);
        FilterKeyPoint(s32_KeyPointIdx);

        if (s32_KeyPointIdx != -1)
        {
            s32_EntranceCheckBuffer++;

            if (s32_EntranceCheckBuffer == BUFFER_SIZE)
            {
                s32_EntranceCheckBuffer = 0;
                st_Planning.b_StaySignal = true;
                s32_Sequence = Sequence::SAVE_KEY_POINT;  // Jump to next sequence
            }
        }
        else
        {
            if (s32_EntranceCheckBuffer > 0)
            {
                s32_EntranceCheckBuffer--;
            }
        }
	}
	else
	{
		cout << ">>> LANE DETECTION FAILED <<<" << endl;
	}
}

void SaveKeyPoint(float32_t &f32_LaneIncline,
                  float32_t &f32_LaneIntercept,
                  float32_t &f32_LaneHeading_rad,
                  int32_t &s32_IsLaneFiltered,
                  int32_t &s32_KeyPointIdx,
                  LiDAR_t &st_LiDAR,
                  Planning_t &st_Planning)
{
    cout << "SEQ: SAVE_KEY_POINT" << endl;

    s32_IsLaneFiltered = FilterParkingLane(f32_LaneIncline,
                                           f32_LaneIntercept,
                                           f32_LaneHeading_rad,
                                           st_LiDAR);

    if (s32_IsLaneFiltered == 0)
	{
        FilterParkingObs(f32_LaneIncline,
                         f32_LaneIntercept,
			             st_LiDAR);
        FilterKeyPoint(s32_KeyPointIdx);

        if (s32_KeyPointIdx != -1)
        {
            vf32_KeyPointStack.push_back({vf32_ObsInfo[s32_KeyPointIdx][2], vf32_ObsInfo[s32_KeyPointIdx][3]});
            s32_EntranceCheckBuffer++;

            if (s32_EntranceCheckBuffer == BUFFER_SIZE)
            {
                float32_t f32_KeyPointE = 0.0f;
                float32_t f32_KeyPointN = 0.0f;

                for (int32_t s32_i = 0; s32_i < BUFFER_SIZE; s32_i++)
                {
                    f32_KeyPointE += vf32_KeyPointStack[s32_i][0];
                    f32_KeyPointN += vf32_KeyPointStack[s32_i][1];
                }

                f32_KeyPointE = f32_KeyPointE / (float)BUFFER_SIZE;
                f32_KeyPointN = f32_KeyPointN / (float)BUFFER_SIZE;

                vf32_KeyPoint.push_back(f32_KeyPointE);
                vf32_KeyPoint.push_back(f32_KeyPointN);

                st_Planning.b_StaySignal = false;
                st_Planning.b_SlowSignal = true;
                s32_Sequence = Sequence::GO_STRAIGHT;  // Jump to next sequence
            }
        }
	}
	else
	{
		cout << ">>> LANE DETECTION FAILED <<<" << endl;
	}
}

void GoStraight(GPSIMU_t &st_GPSIMU,
                Planning_t &st_Planning)
{
    cout << "SEQ: GO_STRAIGHT" << endl;

    if (Normalize2D(st_GPSIMU.f32_E, st_GPSIMU.f32_N, vf32_KeyPoint[0], vf32_KeyPoint[1]) < STRAIGHT_STOP_DIST)
    {
        st_Planning.st_LocalPath.s32_PathNum = 0;
        s32_Sequence = Sequence::SET_DIAGONAL;  // Jump to next sequence
    }
}

void SetDiagonal(GPSIMU_t &st_GPSIMU,
                 Planning_t &st_Planning)
{
    cout << "SEQ: SET_DIAGONAL" << endl;

    if (st_Planning.st_LocalPath.s32_PathNum == 1)
    {
        DiagonalOutPathGen(DIAGONAL_PATH_DIST, st_Planning);
    }

    if (GetGlobalDist(st_GPSIMU.f32_E, st_GPSIMU.f32_N) < 0.5f)
    {
        s32_IsLaneCrossed = 1;
    }

    if (s32_IsLaneCrossed == 1 &&
        Normalize2D(st_GPSIMU.f32_E, st_GPSIMU.f32_N, vf32_KeyPoint[0], vf32_KeyPoint[1]) > DIAGONAL_STOP_DIST)
    {
        st_Planning.b_StaySignal = true;
        st_Planning.b_SlowSignal = false;
        s32_Sequence = Sequence::CHANGE_GEARMODE_R;  // Jump to next sequence
    }
}

void ChangeGearModeR(Planning_t &st_Planning)
{
    cout << "SEQ: CHANGE_GEARMODE_R" << endl;

    st_Planning.b_StaySignal = true;
    st_Planning.s32_GearSignal = 2;

    s32_GearmodeChangeCnt++;
    if (s32_GearmodeChangeCnt == 40 * GEARMODE_R_CHANGE_TIME)  // Hz
    {
        s32_GearmodeChangeCnt = 0;
        st_Planning.st_LocalPath.s32_PathNum = 0;
        st_Planning.b_StaySignal = false;
        st_Planning.b_SlowSignal = true;
        s32_Sequence = Sequence::GO_BACKWARD;  // Jump to next sequence
    }
}

void GoBackward(GPSIMU_t &st_GPSIMU,
                Planning_t &st_Planning)
{
    cout << "SEQ: GO_BACKWARD" << endl;

    if (st_Planning.st_LocalPath.s32_PathNum == 1)
    {
        DiagonalInPathGen(DIAGONAL_PATH_DIST, st_Planning);
    }

    if (GetGlobalDist(st_GPSIMU.f32_E, st_GPSIMU.f32_N) > PARKING_IN_STOP_DIST)
    {
        st_Planning.st_LocalPath.s32_PathNum = 0;
        s32_Sequence = Sequence::PARALLEL_PARKING;  // Jump to next sequence
    }
}

void ParallelParking(GPSIMU_t &st_GPSIMU,
                     Planning_t &st_Planning,
                     Camera_t &st_Camera)
{
    cout << "SEQ: PARALLEL_PARKING" << endl;

    if (st_Planning.st_LocalPath.s32_PathNum == 1)
    {
        ParallelPathGen(KEYPOINT_PARALLEL_PATH_DIST, st_Planning);
    }

    if (st_Camera.f32_BackLaneIntercept > 10.0f && -0.1f < st_Camera.f32_BackLaneSlope && st_Camera.f32_BackLaneSlope < 0.1f) 
    {
        st_Planning.b_StaySignal = true;
        st_Planning.b_SlowSignal = false;
        s32_Sequence = Sequence::CHANGE_GEARMODE_D;  // Jump to next sequence
    }
}

void ChangeGearModeD(Planning_t &st_Planning)
{
    cout << "SEQ: CHANGE_GEARMODE_D" << endl;

    st_Planning.b_StaySignal = true;
    st_Planning.s32_GearSignal = 0;

    s32_GearmodeChangeCnt++;
    if (s32_GearmodeChangeCnt == 40 * GEARMODE_D_CHANGE_TIME)  // Hz
    {
        st_Planning.st_LocalPath.s32_PathNum = 0;
        st_Planning.b_StaySignal = false;
        st_Planning.b_SlowSignal = true;
        s32_Sequence = Sequence::ESCAPE;  // Jump to next sequence
    }
}

void Escape(GPSIMU_t &st_GPSIMU,
            Planning_t &st_Planning)
{
    cout << "SEQ: ESCAPE" << endl;

    if (st_Planning.st_LocalPath.s32_PathNum == 1)
    {
        EscapePathGen(st_GPSIMU, st_Planning);
    }

    if (GetGlobalDist(st_GPSIMU.f32_E, st_GPSIMU.f32_N) < ESCAPE_DIST)
    {
        st_Planning.st_LocalPath.s32_PathNum = 0;
        st_Planning.b_SlowSignal = false;
        s32_IsParkingEnded = 1;
    }
}

void VisualizeParking(float32_t &f32_LaneIncline,
					  float32_t &f32_LaneIntercept,
                      float32_t &f32_LaneHeading_rad,
					  int32_t &s32_IsLaneFiltered,
                      int32_t &s32_KeyPointIdx,
                      GPSIMU_t &st_GPSIMU,
					  Planning_t &st_Planning)
{
	// visualize vehicle
	VisualizePoint({0.0f, 0.0f}, Color::GREEN, 0.5f);

	// visualize road
    if (ERP_MODE)
    {
        if (s32_IsLaneFiltered == 0)
        {
            vector<float32_t> vf32_LaneStart = {0.0f, f32_LaneIntercept};
            vector<float32_t> vf32_LaneEnd = {10.0f, 10.0f * f32_LaneIncline + f32_LaneIntercept};

            vector<float32_t> vf32_LeftBandStart = {0.0f, f32_LaneIntercept - (ROAD_WIDTH + BAND_WIDTH / 2.0f) / cos(f32_LaneHeading_rad)};
            vector<float32_t> vf32_LeftBandEnd = {10.0f, 10.0f * f32_LaneIncline + f32_LaneIntercept - (ROAD_WIDTH + BAND_WIDTH / 2.0f) / cos(f32_LaneHeading_rad)};

            vector<float32_t> vf32_RightBandStart = {0.0f, f32_LaneIntercept - (ROAD_WIDTH - BAND_WIDTH / 2.0f) / cos(f32_LaneHeading_rad)};
            vector<float32_t> vf32_RightBandEnd = {10.0f, 10.0f * f32_LaneIncline + f32_LaneIntercept - (ROAD_WIDTH - BAND_WIDTH / 2.0f) / cos(f32_LaneHeading_rad)};

            VisualizeLine(vf32_LaneStart, vf32_LaneEnd, Color::YELLOW, 0.2f);
            VisualizeLine(vf32_LeftBandStart, vf32_LeftBandEnd, Color::GRAY, 0.2f);
            VisualizeLine(vf32_RightBandStart, vf32_RightBandEnd, Color::GRAY, 0.2f);
        }
    }
    else
    {
        vector<float32_t> vf32_LeftBandStart = {0.0f, -MORAI_ROI_OFFSET + BAND_WIDTH / 2.0f};
        vector<float32_t> vf32_LeftBandEnd = {10.0f, -MORAI_ROI_OFFSET + BAND_WIDTH / 2.0f};

        vector<float32_t> vf32_RightBandStart = {0.0f, -MORAI_ROI_OFFSET - BAND_WIDTH / 2.0f};
        vector<float32_t> vf32_RightBandEnd = {10.0f, -MORAI_ROI_OFFSET - BAND_WIDTH / 2.0f};

        VisualizeLine(vf32_LeftBandStart, vf32_LeftBandEnd, Color::GRAY, 0.2f);
        VisualizeLine(vf32_RightBandStart, vf32_RightBandEnd, Color::GRAY, 0.2f);
    }
	
	// visualize obstacle
	if (s32_IsLaneFiltered == 0 && vf32_ObsInfo.size() > 0)
	{
		for (int32_t s32_i = 0; s32_i < vf32_ObsInfo.size(); s32_i++)
		{
            if (s32_KeyPointIdx != -1 && s32_i == s32_KeyPointIdx)
            {
                VisualizePoint({vf32_ObsInfo[s32_i][0], vf32_ObsInfo[s32_i][1]}, Color::BLUE, 1.0f);
            }
            else
            {
                VisualizePoint({vf32_ObsInfo[s32_i][0], vf32_ObsInfo[s32_i][1]}, Color::ORANGE, 1.0f);
            }
		}
	}
}

/**************************************************/

void Parking(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Camera_t &st_Camera)
{
    float32_t f32_LaneIncline, f32_LaneIntercept, f32_LaneHeading_rad;
    int32_t s32_IsLaneFiltered;
    int32_t s32_KeyPointIdx;

    vf32_ObsInfo.clear();

    if (s32_IsParkingEnded == 1)
    {
        cout << "SEQ: PARKING_ENDED" << endl;
        st_Planning.st_Mission.s_SelectName = "TRACKING";
    }

    if (s32_IsMapLoaded == 0)
    {
        st_Planning.st_LocalPath.s32_PathNum = 0;

        for (int32_t s32_i = 0; s32_i < c_MISSIONMAXSIZE; s32_i++)
        {
            if (st_Planning.st_Mission.ars_Names[s32_i] == "PARKING")
            {
                s32_GlobalStartIdx = st_Planning.st_Mission.arp_Idx[s32_i].first;
                s32_GlobalEndIdx = st_Planning.st_Mission.arp_Idx[s32_i].second;

                vf32_GlobalStart = {st_Planning.st_GlobalPath.arf32_E[s32_GlobalStartIdx], st_Planning.st_GlobalPath.arf32_N[s32_GlobalStartIdx]};
                vf32_GlobalEnd = {st_Planning.st_GlobalPath.arf32_E[s32_GlobalEndIdx], st_Planning.st_GlobalPath.arf32_N[s32_GlobalEndIdx]};

                f32_GlobalIncline = (vf32_GlobalEnd[1] - vf32_GlobalStart[1]) / (vf32_GlobalEnd[0] - vf32_GlobalStart[0]);
                f32_GlobalIntercept = vf32_GlobalStart[1] - f32_GlobalIncline * vf32_GlobalStart[0];
                f32_GlobalDirection_rad = -1.0f * atan((vf32_GlobalEnd[1] - vf32_GlobalStart[1]) / (vf32_GlobalEnd[0] - vf32_GlobalStart[0])) - M_PI / 2.0f;
                
                s32_IsMapLoaded = 1;
                break;
            }
        }
        
        if (s32_IsMapLoaded == 0)
        {
            cout << ">>> LOADING MAP FAILED <<<" << endl;
            return;
        }
    }

    switch (s32_Sequence) {
        case Sequence::SCAN_AREA:
            ScanArea(f32_LaneIncline,
                     f32_LaneIntercept,
                     f32_LaneHeading_rad,
                     s32_IsLaneFiltered,
                     s32_KeyPointIdx,
                     st_LiDAR,
                     st_GPSIMU,
                     st_Planning);
            break;

        case Sequence::SAVE_KEY_POINT:
            SaveKeyPoint(f32_LaneIncline,
                         f32_LaneIntercept,
                         f32_LaneHeading_rad,
                         s32_IsLaneFiltered,
                         s32_KeyPointIdx,
                         st_LiDAR,
                         st_Planning);
            break;

        case Sequence::GO_STRAIGHT:
            GoStraight(st_GPSIMU,
                       st_Planning);
            break;

        case Sequence::SET_DIAGONAL:
            SetDiagonal(st_GPSIMU,
                        st_Planning);
            break;

        case Sequence::CHANGE_GEARMODE_R:
            ChangeGearModeR(st_Planning);
            break;

        case Sequence::GO_BACKWARD:
            GoBackward(st_GPSIMU,
                       st_Planning);
            break;

        case Sequence::PARALLEL_PARKING:
            ParallelParking(st_GPSIMU,
                            st_Planning,
                            st_Camera);
            break;

        case Sequence::CHANGE_GEARMODE_D:
            ChangeGearModeD(st_Planning);
            break;

        case Sequence::ESCAPE:
            Escape(st_GPSIMU,
                   st_Planning);
            break;

        default:
            cout << "SEQ: ERROR" << endl;
            break;
    }

    if (st_Planning.st_LocalPath.s32_PathNum == 0)
    {
        st_Planning.st_LocalPath.arf32_N[0] = 0.0f;
        st_Planning.st_LocalPath.arf32_E[0] = 0.0f;
        st_Planning.st_LocalPath.s32_PathNum++;
    }

    if (s32_Sequence == Sequence::SCAN_AREA || s32_Sequence == Sequence::SAVE_KEY_POINT)
    {
        VisualizeParking(f32_LaneIncline,
                     f32_LaneIntercept,
                     f32_LaneHeading_rad,
                     s32_IsLaneFiltered,
                     s32_KeyPointIdx,
                     st_GPSIMU,
                     st_Planning);
    }

    // cout << "==================================================" << endl;
    // cout << "slope: " << st_Camera.f32_BackLaneSlope << " intercept: " << st_Camera.f32_BackLaneIntercept << endl;
    // cout << "==================================================" << endl;
}

// s32_GlobalNearIdx: 10300
// Speed: 10
// Slow speed: 5
