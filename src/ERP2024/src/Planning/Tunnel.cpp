#include <Planning/Planning.hpp>

bool b_StraightMode = false;
extern bool StraightSignalFlag;
int32_t aaacnt = 0;
bool begin_cnt = false;

bool b_LaneBasedMode = false;
int32_t s32_ModeSwitchingBuffer = 0;  // max value 10, lane based mode when bigger than 5

vector<vector<float32_t>> vf32_StaticObsInfo;  // {EgoInclineDist, RightWallDist}
vector<vector<float32_t>> vf32_DynamicObsInfo;  // {EgoInclineDist, RightWallDist}


float32_t GetRightWallDist(float32_t f32_X,
						   float32_t f32_Y,
						   float32_t &f32_TunnelIncline,
						   float32_t &f32_TunnelRightIntercept)
{
	// distance between a point and right wall
	// (ax + by + c = 0) && (x0, y0)
	return abs(f32_TunnelIncline * f32_X - f32_Y + f32_TunnelRightIntercept) / sqrt(pow(f32_TunnelIncline, 2) + 1.0f);
}

float32_t GetInclineDist(float32_t f32_X1,
						 float32_t f32_Y1,
						 float32_t f32_X2,
						 float32_t f32_Y2,
						 float32_t &f32_TunnelIncline,
						 float32_t &f32_TunnelRightIntercept)
{
	// distance betweem two points toward wall incline direction
	return sqrt(pow(Normalize2D(f32_X1, f32_Y1, f32_X2, f32_Y2), 2) -
				pow(GetRightWallDist(f32_X1, f32_Y1, f32_TunnelIncline, f32_TunnelRightIntercept) -
					GetRightWallDist(f32_X2, f32_Y2, f32_TunnelIncline, f32_TunnelRightIntercept), 2));
}

Point2D_t RotatePoint(float32_t f32_FixedX,
					  float32_t f32_FixedY,
					  float32_t f32_TargetX,
					  float32_t f32_TargetY,
					  float32_t &f32_Heading_rad,
					  bool b_Reverse)
{
	Point2D_t st_Point;

	st_Point.f32_X = (f32_TargetX - f32_FixedX) * cos((b_Reverse ? 1.0f : -1.0f) * f32_Heading_rad) -
					 (f32_TargetY - f32_FixedY) * sin((b_Reverse ? 1.0f : -1.0f) * f32_Heading_rad) + f32_FixedX;
	st_Point.f32_Y = (f32_TargetX - f32_FixedX) * sin((b_Reverse ? 1.0f : -1.0f) * f32_Heading_rad) +
					 (f32_TargetY - f32_FixedY) * cos((b_Reverse ? 1.0f : -1.0f) * f32_Heading_rad) + f32_FixedY;

	return st_Point;
}

int32_t FilterWall(float32_t &f32_WallIncline,
			   float32_t &f32_RightIntercept,
			   float32_t &f32_LeftIntercept,
			   LiDAR_t &st_LiDAR,
			   Tunnel_t &st_Tunnel)
{
	if (-1e-3f < st_LiDAR.arf32_TunnelYIntercept[0] && st_LiDAR.arf32_TunnelYIntercept[0] < 1e-3f &&
		-1e-3f < st_LiDAR.arf32_TunnelYIntercept[1] && st_LiDAR.arf32_TunnelYIntercept[1] < 1e-3f)
	{
		cout << "EXCEPTION: line count is not 2" << endl;
		return 1;
	}

	if (abs(st_LiDAR.arf32_TunnelSlope[0] - st_LiDAR.arf32_TunnelSlope[1]) > st_Tunnel.f32_InclineDiff)
	{
		cout << "EXCEPTION: incline diff out of range" << endl;
		return 1;
	}

	f32_WallIncline = (st_LiDAR.arf32_TunnelSlope[0] + st_LiDAR.arf32_TunnelSlope[1]) / 2.0f;

	if (abs(st_LiDAR.arf32_TunnelYIntercept[0] - st_LiDAR.arf32_TunnelYIntercept[1]) < st_Tunnel.f32_WallWidth)
	{
		cout << "EXCEPTION: tunnel width out of range" << endl;
		return 1;
	}

	if (st_LiDAR.arf32_TunnelYIntercept[0] < st_LiDAR.arf32_TunnelYIntercept[1])
	{
		f32_RightIntercept = st_LiDAR.arf32_TunnelYIntercept[0];
		f32_LeftIntercept = st_LiDAR.arf32_TunnelYIntercept[1];
	}
	else
	{
		f32_RightIntercept = st_LiDAR.arf32_TunnelYIntercept[1];
		f32_LeftIntercept = st_LiDAR.arf32_TunnelYIntercept[0];
	}

	return 0;
}

int32_t FilterLane(float32_t &f32_LaneIncline,
			   float32_t &f32_LaneIntercept,
			   LiDAR_t &st_LiDAR)
{
	if (99.999f < st_LiDAR.arf32_MiddleLaneYIntercept && st_LiDAR.arf32_MiddleLaneYIntercept < 100.001f)
	{
		cout << "EXCEPTION: lane is not detected" << endl;
		return 1;
	}

	f32_LaneIncline = st_LiDAR.arf32_MiddleLaneSlope;
	f32_LaneIntercept = st_LiDAR.arf32_MiddleLaneYIntercept;

	return 0;
}

void FilterObs(float32_t &f32_TunnelIncline,
			   float32_t &f32_TunnelRightIntercept,
			   LiDAR_t &st_LiDAR,
			   Tunnel_t &st_Tunnel)
{
	if (st_LiDAR.s32_ObjectNum > 0)
	{
		float32_t f32_Obs_RightWallDist, f32_Obs_EgoInclineDist;

		for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
		{
			if (st_LiDAR.st_XY[s32_i].f32_X < 0.0f)
			{
				cout << "EXCEPTION: object filtered by range of interest" << endl;
				continue;
			}

			f32_Obs_EgoInclineDist = GetInclineDist(0.0f, 0.0f, st_LiDAR.st_XY[s32_i].f32_X, st_LiDAR.st_XY[s32_i].f32_Y, f32_TunnelIncline, f32_TunnelRightIntercept);
			f32_Obs_RightWallDist = GetRightWallDist(st_LiDAR.st_XY[s32_i].f32_X, st_LiDAR.st_XY[s32_i].f32_Y, f32_TunnelIncline, f32_TunnelRightIntercept);
			
			if (f32_Obs_RightWallDist + st_LiDAR.arf32_Size[s32_i] / 2.0f < st_Tunnel.f32_RoadCenter - st_Tunnel.f32_RoadWidth / 2.0f ||
				st_Tunnel.f32_RoadCenter + st_Tunnel.f32_RoadWidth / 2.0f < f32_Obs_RightWallDist - st_LiDAR.arf32_Size[s32_i] / 2.0f)
			{
				vf32_DynamicObsInfo.push_back({f32_Obs_EgoInclineDist, f32_Obs_RightWallDist});
			}
			else
			{
				vf32_StaticObsInfo.push_back({f32_Obs_EgoInclineDist, f32_Obs_RightWallDist});
			}
		}

		if (vf32_StaticObsInfo.size() > 0)
		{
			sort(vf32_StaticObsInfo.begin(), vf32_StaticObsInfo.end(), [](const vector<float32_t> &a, const vector<float32_t> &b)
			{ return a[0] < b[0]; });
		}
		if (vf32_DynamicObsInfo.size() > 0)
		{
			sort(vf32_DynamicObsInfo.begin(), vf32_DynamicObsInfo.end(), [](const vector<float32_t> &a, const vector<float32_t> &b)
			{ return a[0] < b[0]; });
		}
	}
}

int32_t FilterTunnelOut(float32_t &f32_TunnelOutDist,
						float32_t &f32_TunnelIncline,
						float32_t &f32_TunnelRightIntercept,
						LiDAR_t &st_LiDAR)
{
	float32_t f32_XMean, f32_YMean;

	if (st_LiDAR.st_TunnelOutPoint[0].f32_X > 0.0f && st_LiDAR.st_TunnelOutPoint[1].f32_X > 0.0f)
	{
		f32_XMean = (st_LiDAR.st_TunnelOutPoint[0].f32_X + st_LiDAR.st_TunnelOutPoint[1].f32_X) / 2.0f;
		f32_YMean = (st_LiDAR.st_TunnelOutPoint[0].f32_Y + st_LiDAR.st_TunnelOutPoint[1].f32_Y) / 2.0f;
	
		f32_TunnelOutDist = GetInclineDist(0.0f, 0.0f, f32_XMean, f32_YMean, f32_TunnelIncline, f32_TunnelRightIntercept);

		return 0;
	}

	return 1;
}

void InitPath(float32_t f32_InitRightWallDist,
			  float32_t &f32_Heading_rad,
			  float32_t &f32_Ego_RightWallDist,
			  Planning_t &st_Planning)
{
	int32_t s32_PathNum = st_Planning.st_LocalPath.s32_PathNum;

	st_Planning.st_LocalPath.arf32_N[s32_PathNum] = -sin(f32_Heading_rad) * (f32_Ego_RightWallDist - f32_InitRightWallDist);
	st_Planning.st_LocalPath.arf32_E[s32_PathNum] = -cos(f32_Heading_rad) * (f32_Ego_RightWallDist - f32_InitRightWallDist);
	s32_PathNum++;

	st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;

	return;
}

void StraightPathGen(float32_t f32_PathLen,
					 float32_t &f32_TunnelIncline,
					 Planning_t &st_Planning)
{
	int32_t s32_PathNum = st_Planning.st_LocalPath.s32_PathNum;
	int32_t s32_cnt = 0;

	for (int32_t s32_i = 0; s32_i < f32_PathLen / st_Planning.st_Tunnel.f32_WayPointDist; s32_i++)
	{
		st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Planning.st_LocalPath.arf32_N[s32_PathNum - 1] +
														st_Planning.st_Tunnel.f32_WayPointDist / sqrt(1.0f + pow(f32_TunnelIncline, 2));
		st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Planning.st_LocalPath.arf32_E[s32_PathNum - 1] +
														st_Planning.st_Tunnel.f32_WayPointDist / sqrt(1.0f + pow(f32_TunnelIncline, 2)) * f32_TunnelIncline;
		s32_PathNum++;
		s32_cnt++;
	}

	float32_t f32_RemainLen = f32_PathLen - (float)s32_cnt * st_Planning.st_Tunnel.f32_WayPointDist;

	st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Planning.st_LocalPath.arf32_N[s32_PathNum - 1] +
													f32_RemainLen / sqrt(1.0f + pow(f32_TunnelIncline, 2));
	st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Planning.st_LocalPath.arf32_E[s32_PathNum - 1] +
													f32_RemainLen / sqrt(1.0f + pow(f32_TunnelIncline, 2)) * f32_TunnelIncline;
	s32_PathNum++;
	st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;

	return;
}

void CurvePathGen(Point2D_t st_StartPoint,
				  Point2D_t st_EndPoint,
				  float32_t &f32_Heading_rad,
				  Planning_t &st_Planning)
{
	int32_t s32_PathNum = st_Planning.st_LocalPath.s32_PathNum;

	// y = alpha * (x - x_start) ^ 2 * (x - (3 * x_end - x_start) / 2) + y_start
	float32_t f32_Alpha = -2.0f * (st_EndPoint.f32_Y - st_StartPoint.f32_Y) / pow(st_EndPoint.f32_X - st_StartPoint.f32_X, 3);
	float32_t f32_TargetX, f32_TargetY;
	Point2D_t st_Point;

	for (float32_t f32_Val = st_StartPoint.f32_X + st_Planning.st_Tunnel.f32_WayPointDist;
		 f32_Val < st_EndPoint.f32_X; f32_Val += st_Planning.st_Tunnel.f32_WayPointDist)
	{
		f32_TargetX = f32_Val;
		f32_TargetY = f32_Alpha * pow(f32_Val - st_StartPoint.f32_X, 2) *
					  (f32_Val - (3.0f * st_EndPoint.f32_X - st_StartPoint.f32_X) / 2.0f) +
					  st_StartPoint.f32_Y;
		st_Point = RotatePoint(0.0f, 0.0f, f32_TargetX, f32_TargetY, f32_Heading_rad, false);

		st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Point.f32_X;
		st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Point.f32_Y;
		s32_PathNum++;
	}

	st_Point = RotatePoint(0.0f, 0.0f, st_EndPoint.f32_X, st_EndPoint.f32_Y, f32_Heading_rad, false);

	st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Point.f32_X;
	st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Point.f32_Y;
	s32_PathNum++;
	st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;

	return;
}

void LocalPathGen(float32_t &f32_TunnelIncline,
				  float32_t &f32_Heading_rad,
				  float32_t &f32_Ego_RightWallDist,
				  Planning_t &st_Planning)
{
	Point2D_t st_StartPoint, st_EndPoint;
	float32_t f32_Obs_EgoInclineDist, f32_Obs_RightWallDist;
	float32_t f32_TargetRightWallDist;

	if (vf32_StaticObsInfo.size() == 0)
	{
		f32_TargetRightWallDist = st_Planning.st_Tunnel.f32_RoadCenter;

		InitPath(f32_Ego_RightWallDist, f32_Heading_rad, f32_Ego_RightWallDist, st_Planning);

		st_StartPoint.f32_X = 0.0f;
		st_StartPoint.f32_Y = 0.0f;
		st_EndPoint.f32_X = abs(f32_TargetRightWallDist - f32_Ego_RightWallDist);
		st_EndPoint.f32_Y = f32_TargetRightWallDist - f32_Ego_RightWallDist;
		CurvePathGen(st_StartPoint, st_EndPoint, f32_Heading_rad, st_Planning);

		StraightPathGen(15.0f - st_EndPoint.f32_X, f32_TunnelIncline, st_Planning);
	}
	else
	{
		f32_Obs_EgoInclineDist = vf32_StaticObsInfo[0][0];
		f32_Obs_RightWallDist = vf32_StaticObsInfo[0][1];

		if (abs(vf32_StaticObsInfo[0][1] - st_Planning.st_Tunnel.f32_RoadCenter) < 0.75f)
		{
			f32_TargetRightWallDist = st_Planning.st_Tunnel.f32_RoadCenter;

			InitPath(f32_Ego_RightWallDist, f32_Heading_rad, f32_Ego_RightWallDist, st_Planning);

			st_StartPoint.f32_X = 0.0f;
			st_StartPoint.f32_Y = 0.0f;
			st_EndPoint.f32_X = abs(f32_TargetRightWallDist - f32_Ego_RightWallDist);
			st_EndPoint.f32_Y = f32_TargetRightWallDist - f32_Ego_RightWallDist;
			CurvePathGen(st_StartPoint, st_EndPoint, f32_Heading_rad, st_Planning);

			StraightPathGen(f32_Obs_EgoInclineDist - 1.0f, f32_TunnelIncline, st_Planning);  // not so accurate but...

			return;
		}

		f32_TargetRightWallDist = st_Planning.st_Tunnel.f32_RoadCenter +
								  st_Planning.st_Tunnel.f32_AvoidOffset * (f32_Obs_RightWallDist < st_Planning.st_Tunnel.f32_RoadCenter ? 1.0f : -1.0f);

		InitPath(f32_Ego_RightWallDist, f32_Heading_rad, f32_Ego_RightWallDist, st_Planning);

		st_StartPoint.f32_X = 0.0f;
		st_StartPoint.f32_Y = 0.0f;
		st_EndPoint.f32_X = abs(f32_TargetRightWallDist - f32_Ego_RightWallDist);
		st_EndPoint.f32_Y = f32_TargetRightWallDist - f32_Ego_RightWallDist;
		CurvePathGen(st_StartPoint, st_EndPoint, f32_Heading_rad, st_Planning);

		StraightPathGen(f32_Obs_EgoInclineDist - st_EndPoint.f32_X + st_Planning.st_Tunnel.f32_AvoidMargin, f32_TunnelIncline, st_Planning);

		if (vf32_StaticObsInfo.size() > 1)
		{
			for (int32_t s32_ObsCnt = 1; s32_ObsCnt < vf32_StaticObsInfo.size(); s32_ObsCnt++)
			{
				f32_Obs_EgoInclineDist = vf32_StaticObsInfo[s32_ObsCnt][0];
				f32_Obs_RightWallDist = vf32_StaticObsInfo[s32_ObsCnt][1];
				f32_TargetRightWallDist = st_Planning.st_Tunnel.f32_RoadCenter +
										  st_Planning.st_Tunnel.f32_AvoidOffset * (f32_Obs_RightWallDist < st_Planning.st_Tunnel.f32_RoadCenter ? 1.0f : -1.0f);

				st_StartPoint = RotatePoint(0.0f, 0.0f,
											st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum - 1],
											st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum - 1],
											f32_Heading_rad,
											true);
				st_EndPoint.f32_X = st_StartPoint.f32_X + st_Planning.st_Tunnel.f32_AvoidOffset * 2.0f;
				st_EndPoint.f32_Y = f32_TargetRightWallDist - f32_Ego_RightWallDist;
				CurvePathGen(st_StartPoint, st_EndPoint, f32_Heading_rad, st_Planning);

				StraightPathGen(f32_Obs_EgoInclineDist - st_EndPoint.f32_X + st_Planning.st_Tunnel.f32_AvoidMargin, f32_TunnelIncline, st_Planning);
			}
		}

		f32_TargetRightWallDist = st_Planning.st_Tunnel.f32_RoadCenter;

		st_StartPoint = RotatePoint(0.0f, 0.0f,
									st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum - 1],
									st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum - 1],
									f32_Heading_rad,
									true);
		st_EndPoint.f32_X = st_StartPoint.f32_X + st_Planning.st_Tunnel.f32_AvoidOffset * 2.0f;
		st_EndPoint.f32_Y = f32_TargetRightWallDist - f32_Ego_RightWallDist;
		CurvePathGen(st_StartPoint, st_EndPoint, f32_Heading_rad, st_Planning);

		StraightPathGen(15.0f, f32_TunnelIncline, st_Planning);
	}
}

void VisualizeTunnel(float32_t &f32_WallIncline,
					 float32_t &f32_RightIntercept,
					 float32_t &f32_LeftIntercept,
					 float32_t &f32_LaneIncline,
					 float32_t &f32_LaneIntercept,
					 float32_t &f32_Heading_rad,
					 float32_t &f32_Ego_RightWallDist,
					 int32_t &s32_IsWallFiltered,
					 int32_t &s32_IsLaneFiltered,
					 Planning_t &st_Planning,
					 Tunnel_t &st_Tunnel)
{
	// visualize vehicle
	VisualizePoint({0.0f, 0.0f}, Color::GREEN, 0.5f);

	// visualize wall
	if (s32_IsWallFiltered == 0)
	{
		vector<float32_t> vf32_RightWallStart = {0.0f, f32_RightIntercept};
		vector<float32_t> vf32_RightWallEnd = {15.0f, 15.0f * f32_WallIncline + f32_RightIntercept};
		vector<float32_t> vf32_LeftWallStart = {0.0f, f32_LeftIntercept};
		vector<float32_t> vf32_LeftWallEnd = {15.0f, 15.0f * f32_WallIncline + f32_LeftIntercept};

		if (b_LaneBasedMode == false)
		{
			VisualizeLine(vf32_RightWallStart, vf32_RightWallEnd, Color::WHITE, 0.2f);
			VisualizeLine(vf32_LeftWallStart, vf32_LeftWallEnd, Color::WHITE, 0.2f);
		}
		else
		{
			VisualizeLine(vf32_RightWallStart, vf32_RightWallEnd, Color::GRAY, 0.2f);
			VisualizeLine(vf32_LeftWallStart, vf32_LeftWallEnd, Color::GRAY, 0.2f);
		}
	}

	// visualize lane
	if (s32_IsLaneFiltered == 0)
	{
		vector<float32_t> vf32_RightLaneStart = {0.0f, f32_LaneIntercept - st_Tunnel.f32_RoadWidth / cos(f32_Heading_rad)};
		vector<float32_t> vf32_RightLaneEnd = {5.0f, 5.0f * f32_LaneIncline + f32_LaneIntercept - st_Tunnel.f32_RoadWidth / cos(f32_Heading_rad)};
		vector<float32_t> vf32_LeftLaneStart = {0.0f, f32_LaneIntercept};
		vector<float32_t> vf32_LeftLaneEnd = {5.0f, 5.0f * f32_LaneIncline + f32_LaneIntercept};

		if (b_LaneBasedMode == false)
		{
			VisualizeLine(vf32_RightLaneStart, vf32_RightLaneEnd, Color::GRAY, 0.2f);
			VisualizeLine(vf32_LeftLaneStart, vf32_LeftLaneEnd, Color::GRAY, 0.2f);
		}
		else
		{
			VisualizeLine(vf32_RightLaneStart, vf32_RightLaneEnd, Color::WHITE, 0.2f);
			VisualizeLine(vf32_LeftLaneStart, vf32_LeftLaneEnd, Color::WHITE, 0.2f);
		}
	}

	// visualize obstacle
	if ((s32_IsWallFiltered == 0 || s32_IsLaneFiltered == 0) && vf32_StaticObsInfo.size() > 0)
	{
		for (int32_t s32_i = 0; s32_i < vf32_StaticObsInfo.size(); s32_i++)
		{
			Point2D_t st_Obstacle;
			st_Obstacle = RotatePoint(0.0f, 0.0f, vf32_StaticObsInfo[s32_i][0], vf32_StaticObsInfo[s32_i][1] - f32_Ego_RightWallDist, f32_Heading_rad, false);
			VisualizePoint({st_Obstacle.f32_X, st_Obstacle.f32_Y}, Color::ORANGE, 1.0f);
		}
	}
	if ((s32_IsWallFiltered == 0 || s32_IsLaneFiltered == 0) && vf32_DynamicObsInfo.size() > 0)
	{
		for (int32_t s32_i = 0; s32_i < vf32_DynamicObsInfo.size(); s32_i++)
		{
			Point2D_t st_Obstacle;
			st_Obstacle = RotatePoint(0.0f, 0.0f, vf32_DynamicObsInfo[s32_i][0], vf32_DynamicObsInfo[s32_i][1] - f32_Ego_RightWallDist, f32_Heading_rad, false);
			VisualizePoint({st_Obstacle.f32_X, st_Obstacle.f32_Y}, Color::BLUE, 1.0f);
		}
	}

	// visualize path
	VisualizeLocalPath(Color::GREEN, 0.1f, st_Planning);

	return;
}

void MiniVisualizeTunnel(Planning_t &st_Planning)
{
	// visualize vehicle
	VisualizePoint({0.0f, 0.0f}, Color::GREEN, 0.5f);

	// visualize path
	VisualizeLocalPath(Color::GREEN, 0.1f, st_Planning);

	return;
}

void Tunnel(LiDAR_t &st_LiDAR, Planning_t &st_Planning, Tunnel_t &st_Tunnel)
{
	float32_t f32_WallIncline, f32_RightIntercept, f32_LeftIntercept;
	float32_t f32_LaneIncline, f32_LaneIntercept, f32_LaneOffsetIntercept;
	float32_t f32_Heading_rad, f32_Ego_RightWallDist;
	float32_t f32_TunnelOutDist;

	int32_t s32_IsWallFiltered = FilterWall(f32_WallIncline, f32_RightIntercept, f32_LeftIntercept, st_LiDAR, st_Tunnel);
	int32_t s32_IsLaneFiltered = FilterLane(f32_LaneIncline, f32_LaneIntercept, st_LiDAR);
	int32_t s32_IsTunnelOutFiltered;

	if (b_StraightMode)
	{
		cout << "<<<straight mode>>>" << endl;
		StraightSignalFlag = true;
		MiniVisualizeTunnel(st_Planning);
		
		return;
	}

	if (b_LaneBasedMode == false && s32_IsWallFiltered == 0)  /* wall detected */
	{
		f32_Heading_rad = -1.0f * atan(f32_WallIncline);
		f32_Ego_RightWallDist = GetRightWallDist(0.0f, 0.0f, f32_WallIncline, f32_RightIntercept);
		s32_IsTunnelOutFiltered = FilterTunnelOut(f32_TunnelOutDist, f32_WallIncline, f32_RightIntercept, st_LiDAR);

		vf32_StaticObsInfo.clear();
		vf32_DynamicObsInfo.clear();
		FilterObs(f32_WallIncline, f32_RightIntercept, st_LiDAR, st_Tunnel);

		st_Planning.st_LocalPath.s32_PathNum = 0;
		LocalPathGen(f32_WallIncline, f32_Heading_rad, f32_Ego_RightWallDist, st_Planning);
	}
	else if (s32_IsLaneFiltered == 0)  /* lane detected */
	{
		f32_Heading_rad = -1.0f * atan(f32_LaneIncline);
		f32_LaneOffsetIntercept = f32_LaneIntercept - (st_Tunnel.f32_RoadCenter + st_Tunnel.f32_RoadWidth / 2.0f) / cos(f32_Heading_rad);
		f32_Ego_RightWallDist = GetRightWallDist(0.0f, 0.0f, f32_LaneIncline, f32_LaneOffsetIntercept);
		s32_IsTunnelOutFiltered = FilterTunnelOut(f32_TunnelOutDist, f32_LaneIncline, f32_LaneOffsetIntercept, st_LiDAR);

		vf32_StaticObsInfo.clear();
		vf32_DynamicObsInfo.clear();
		FilterObs(f32_LaneIncline, f32_LaneOffsetIntercept, st_LiDAR, st_Tunnel);

		st_Planning.st_LocalPath.s32_PathNum = 0;
		LocalPathGen(f32_LaneIncline, f32_Heading_rad, f32_Ego_RightWallDist, st_Planning);
	}
	else  /* detection failed */
	{
		cout << ">>> DETECTION FAILED <<<" << endl;
		// keep the previous local path

		if (st_Planning.st_LocalPath.s32_PathNum == 0)
		{
			st_Planning.st_LocalPath.arf32_N[0] = 0.0f;
			st_Planning.st_LocalPath.arf32_E[0] = 0.0f;
			st_Planning.st_LocalPath.s32_PathNum++;
		}
	}

	if (s32_IsTunnelOutFiltered == 0)
	{
		if (f32_TunnelOutDist < 10.0f)
		{
			if (s32_ModeSwitchingBuffer < 10)
			{
				s32_ModeSwitchingBuffer++;
			}
		}
		else
		{
			if (s32_ModeSwitchingBuffer > 0)
			{
				s32_ModeSwitchingBuffer--;
			}
		}
	}

	if (s32_ModeSwitchingBuffer > 5)
	{
		b_LaneBasedMode = true;
		st_Planning.b_SlowSignal = true;
	}
	else
	{
		b_LaneBasedMode = false;
		st_Planning.b_SlowSignal = false;
	}

	if (b_LaneBasedMode && s32_IsTunnelOutFiltered == 1)
	{
		begin_cnt = true;
		st_Planning.b_SlowSignal = true;
	}

	if (begin_cnt)
	{
		aaacnt++;
	}

	if (aaacnt > 85)
	{
		StraightSignalFlag = true;
		b_StraightMode = true;
		st_Planning.st_LocalPath.s32_PathNum = 0;
		st_Planning.st_LocalPath.arf32_N[0] = 0.0f;
		st_Planning.st_LocalPath.arf32_E[0] = 0.0f;
		st_Planning.st_LocalPath.s32_PathNum++;

		int32_t s32_PathNum = st_Planning.st_LocalPath.s32_PathNum;
		for (int32_t s32_i = 0; s32_i < 100; s32_i++)
		{
			st_Planning.st_LocalPath.arf32_N[s32_PathNum] = st_Planning.st_LocalPath.arf32_N[s32_PathNum - 1] +
															st_Planning.st_Tunnel.f32_WayPointDist;
			st_Planning.st_LocalPath.arf32_E[s32_PathNum] = st_Planning.st_LocalPath.arf32_E[s32_PathNum - 1];
			s32_PathNum++;
		}
		st_Planning.st_LocalPath.s32_PathNum = s32_PathNum;

		st_Planning.b_StaySignal = false;
		return;
	}

	if (vf32_DynamicObsInfo.size() > 0 && vf32_DynamicObsInfo[0][0] < 10.0f)
	{
		st_Planning.b_SlowSignal = true;
	}
	else
	{
		st_Planning.b_SlowSignal = false;
	}

	st_Planning.b_StaySignal = false;
	if (vf32_StaticObsInfo.size() > 0 && vf32_StaticObsInfo[0][0] < 5.0f)
	{
		if (abs(vf32_StaticObsInfo[0][1] - st_Tunnel.f32_RoadCenter) < 0.75f)
		{
			st_Planning.b_StaySignal = true;
		}
	}

	VisualizeTunnel(f32_WallIncline,
					f32_RightIntercept,
					f32_LeftIntercept,
					f32_LaneIncline,
					f32_LaneIntercept,
					f32_Heading_rad,
					f32_Ego_RightWallDist,
					s32_IsWallFiltered,
					s32_IsLaneFiltered,
					st_Planning,
					st_Tunnel);
}
