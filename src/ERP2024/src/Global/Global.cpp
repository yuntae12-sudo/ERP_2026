#include <Global/Global.hpp>
#include <Planning/Planning.hpp>

float64_t Deg2Rad(float64_t &f64_Deg)
{
    return f64_Deg * M_PI / 180.0;
}
float32_t Deg2Rad(float32_t &f32_Deg)
{
    return f32_Deg * M_PI / 180.0f;
}

float64_t Rad2Deg(float64_t &f64_Rad)
{
    return f64_Rad * 180.0 / M_PI;
}
float32_t Rad2Deg(float32_t &f32_Rad)
{
    return f32_Rad * 180.0f / M_PI;
}

float32_t Scale2D(float32_t &f32_X, float32_t &f32_Y)
{
    return sqrt(pow(f32_X, 2) + pow(f32_Y, 2));
}
float32_t Scale2D(float32_t *arf_X, float32_t *arf_Y, int32_t s32_Idx)
{
    return sqrt(pow(arf_X[s32_Idx], 2) + pow(arf_Y[s32_Idx], 2));
}
float32_t Scale2D(Point2D_t &st_Point2D)
{
    return sqrt(pow(st_Point2D.f32_X, 2) + pow(st_Point2D.f32_Y, 2));
}

float32_t Scale3D(float32_t &f32_X, float32_t &f32_Y, float32_t &f32_Z)
{
    return sqrt(pow(f32_X, 2) + pow(f32_Y, 2) + pow(f32_Z, 2));
}
float32_t Scale3D(float32_t *arf_X, float32_t *arf_Y, float32_t *arf_Z, int32_t s32_Idx)
{
    return sqrt(pow(arf_X[s32_Idx], 2) + pow(arf_Y[s32_Idx], 2) + pow(arf_Z[s32_Idx], 2));
}
float32_t Scale3D(Point3D_t &st_Point3D)
{
    return sqrt(pow(st_Point3D.f32_X, 2) + pow(st_Point3D.f32_Y, 2) + pow(st_Point3D.f32_Z, 2));
}

float32_t Normalize2D(float32_t &f32_X1, float32_t &f32_Y1, float32_t &f32_X2, float32_t &f32_Y2)
{
    return sqrt(pow(f32_X1 - f32_X2, 2) + pow(f32_Y1 - f32_Y2, 2));
}
float32_t Normalize2D(float32_t *arf_X, float32_t *arf_Y, int32_t &s32_Idx1, int32_t &s32_Idx2)
{
    return sqrt(pow(arf_X[s32_Idx1] - arf_X[s32_Idx2], 2) + pow(arf_Y[s32_Idx1] - arf_Y[s32_Idx2], 2));
}
float32_t Normalize2D(float32_t *arf_X, float32_t *arf_Y, int32_t s32_Idx1, float32_t &f32_X, float32_t &f32_Y)
{
    return sqrt(pow(arf_X[s32_Idx1] - f32_X, 2) + pow(arf_Y[s32_Idx1] - f32_Y, 2));
}
float32_t Normalize2D(float32_t *arf_X, float32_t *arf_Y, int32_t s32_Idx1, Point2D_t &st_P2)
{
    return sqrt(pow(arf_X[s32_Idx1] - st_P2.f32_X, 2) + pow(arf_Y[s32_Idx1] - st_P2.f32_Y, 2));
}
float32_t Normalize2D(Point2D_t &st_P1, Point2D_t &st_P2)
{
    return sqrt(pow(st_P1.f32_X - st_P2.f32_X, 2) + pow(st_P1.f32_Y - st_P2.f32_Y, 2));
}
float32_t Normalize2D(std::vector<float32_t> &vf_P1, Point2D_t &st_P2)
{
    return sqrt(pow(vf_P1[0] - st_P2.f32_X, 2) + pow(vf_P1[1] - st_P2.f32_Y, 2));
}
float32_t Normalize2D(std::vector<float32_t> &vf_P1, float32_t &f32_X, float32_t &f32_Y)
{
    return sqrt(pow(vf_P1[0] - f32_X, 2) + pow(vf_P1[1] - f32_Y, 2));
}
float32_t Normalize2D(std::vector<float32_t> &vf_P1, std::vector<float32_t> &vf_P2)
{
    return sqrt(pow(vf_P1[0] - vf_P2[0], 2) + pow(vf_P1[1] - vf_P2[1], 2));
}

float32_t Normalize3D(float32_t &f32_X1, float32_t &f32_Y1, float32_t &f32_Z1, float32_t &f32_X2, float32_t &f32_Y2, float32_t &f32_Z2)
{
    return sqrt(pow(f32_X1 - f32_X2, 2) + pow(f32_Y1 - f32_Y2, 2) + pow(f32_Z1 - f32_Z2, 2));
}
float32_t Normalize3D(float32_t *arf_X, float32_t *arf_Y, float32_t *arf_Z, int32_t &s32_Idx1, int32_t &s32_Idx2)
{
    return sqrt(pow(arf_X[s32_Idx1] - arf_X[s32_Idx2], 2) + pow(arf_Y[s32_Idx1] - arf_Y[s32_Idx2], 2) + pow(arf_Z[s32_Idx1] - arf_Z[s32_Idx2], 2));
}
float32_t Normalize3D(Point3D_t &st_P1, Point3D_t &st_P2)
{
    return sqrt(pow(st_P1.f32_X - st_P2.f32_X, 2) + pow(st_P1.f32_Y - st_P2.f32_Y, 2) + pow(st_P1.f32_Z - st_P2.f32_Z, 2));
}

float32_t KPH2MPS(float32_t f32_KPH)
{
    return f32_KPH / 3.6;
}
float32_t MPS2KPH(float32_t f32_MPS)
{
    return f32_MPS * 3.6;
}

Point2D_t UnitVector(Point2D_t &st_Point2DStart, Point2D_t &st_Point2DEnd)
{
    float32_t f32_Scale = Normalize2D(st_Point2DStart, st_Point2DEnd);
    Point2D_t st_UnitVector = {(st_Point2DEnd.f32_X - st_Point2DStart.f32_X) / f32_Scale,
                               (st_Point2DEnd.f32_Y - st_Point2DStart.f32_Y) / f32_Scale};
    return st_UnitVector;
}
Point2D_t UnitVector(float32_t f32_XStart, float32_t f32_YStart, float32_t f32_XEnd, float32_t f32_YEnd)
{
    float32_t f32_Scale = Normalize2D(f32_XStart, f32_YStart, f32_XEnd, f32_YEnd);
    Point2D_t st_UnitVector = {(f32_XEnd - f32_XStart) / f32_Scale,
                               (f32_YEnd - f32_YStart) / f32_Scale};
    return st_UnitVector;
}

Point2D_t RotatePoint(Point2D_t &st_Point2D, float32_t f32_Degree)
{
    float32_t arf_Rotate[2][2] = {{cos(Deg2Rad(f32_Degree)), -sin(Deg2Rad(f32_Degree))}, {sin(Deg2Rad(f32_Degree)), cos(Deg2Rad(f32_Degree))}};
    Point2D_t st_RotatePoint = {st_Point2D.f32_X * arf_Rotate[0][0] + st_Point2D.f32_Y * arf_Rotate[0][1],
                                st_Point2D.f32_X * arf_Rotate[1][0] + st_Point2D.f32_Y * arf_Rotate[1][1]};
    return st_RotatePoint;
}

bool CrossProduct(Point2D_t &st_Point2DStart, Point2D_t &st_Point2DEnd)
{
    float32_t VectorProduct = st_Point2DStart.f32_X * st_Point2DEnd.f32_Y - st_Point2DEnd.f32_X * st_Point2DStart.f32_Y;

    if(VectorProduct >= 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void GetGapPoints(Planning_t &st_Planning, std::vector<float> &vf_Ref, Point2D_t &st_Direction, int32_t &s32_Iter, float32_t &f32_Gap, std::vector<std::vector<float32_t>> &vf_XY)
{
    for (int32_t s32_i = 0; s32_i < s32_Iter; s32_i++)
    {
        vf_XY.push_back({vf_Ref[0] + f32_Gap * (s32_Iter - s32_i) * st_Direction.f32_X,
                         vf_Ref[1] + f32_Gap * (s32_Iter - s32_i) * st_Direction.f32_Y});
    }
}

void MakePath(Planning_t &st_Planning, int32_t &s32_Start, int32_t &s32_Last, Point2D_t &st_Direction, float32_t &f32_Gap)
{
    for (int32_t s32_i = s32_Start; s32_i < s32_Last; s32_i++)
    {
        st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum] = st_Planning.st_GlobalPath.arf32_E[s32_i] + st_Direction.f32_X * f32_Gap;
        st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum] = st_Planning.st_GlobalPath.arf32_N[s32_i] + st_Direction.f32_Y * f32_Gap;
        st_Planning.st_LocalPath.s32_PathNum++;
    }
}

void MakePath_LiDAR(Planning_t &st_Planning, int32_t &s32_Start, int32_t &s32_Last, Point2D_t &st_Direction, float32_t &f32_Gap)
{
    for (int32_t s32_i = s32_Start; s32_i < s32_Last; s32_i++)
    {
        st_Planning.st_LocalPath.arf32_E[st_Planning.st_LocalPath.s32_PathNum] = st_Planning.st_RefPath.arf32_X[s32_i] + st_Direction.f32_X * f32_Gap;
        st_Planning.st_LocalPath.arf32_N[st_Planning.st_LocalPath.s32_PathNum] = st_Planning.st_RefPath.arf32_Y[s32_i] + st_Direction.f32_Y * f32_Gap;
        st_Planning.st_LocalPath.s32_PathNum++;
    }
}

void LoadData(std::ifstream &st_LoadFile, Path_t &st_Path)
{
    std::string s_FileLine = "";
    float32_t f32_X = 0, f32_Y = 0;
    int32_t s32_Num = 0;
    while (std::getline(st_LoadFile, s_FileLine))
    {
        std::istringstream st_Stream(s_FileLine);
        st_Stream >> f32_X >> f32_Y;
        st_Path.arf32_E[s32_Num] = f32_X;
        st_Path.arf32_N[s32_Num] = f32_Y;
        s32_Num += 1;
    }
    st_Path.s32_PathNum = s32_Num;
    st_LoadFile.close();
}

void LoadIdx(std::ifstream &st_LoadFile, IdxSet_t &st_IdxSet)
{
    std::string s_FileLine = "";
    std::string s_Name = "";
    int32_t s32_Num = 0;
    int32_t s32_Start = 0, s32_End = 0;
    while (std::getline(st_LoadFile, s_FileLine))
    {
        std::istringstream st_Stream(s_FileLine);
        st_Stream >> s_Name >> s32_Start >> s32_End;
        st_IdxSet.ars_Names[s32_Num] = s_Name;
        st_IdxSet.arp_Idx[s32_Num].first = s32_Start;
        st_IdxSet.arp_Idx[s32_Num].second = s32_End;
        s32_Num++;
    }
    st_IdxSet.s32_Num = s32_Num;
}

void LLA2ENU(GPSIMU_t &st_GPSIMU)
{
    float64_t f64_Chi = sqrt(1 - c_N2 * pow(sin(st_GPSIMU.st_WGSRAD.f64_Latitude), 2));
    float64_t f64_Q = (c_EarthRadius / f64_Chi + st_GPSIMU.st_WGSRAD.f64_Altitude) * cos(st_GPSIMU.st_WGSRAD.f64_Latitude);

    float64_t f64_X = f64_Q * cos(st_GPSIMU.st_WGSRAD.f64_Longitude);
    float64_t f64_Y = f64_Q * sin(st_GPSIMU.st_WGSRAD.f64_Longitude);
    float64_t f64_Z = ((c_EarthRadius * (1 - c_N2) / f64_Chi) + st_GPSIMU.st_WGSRAD.f64_Altitude) * sin(st_GPSIMU.st_WGSRAD.f64_Latitude);

    float64_t f64_dX = f64_X - st_GPSIMU.st_RefECEF.f64_X;
    float64_t f64_dY = f64_Y - st_GPSIMU.st_RefECEF.f64_Y;
    float64_t f64_dZ = f64_Z - st_GPSIMU.st_RefECEF.f64_Z;

    st_GPSIMU.f32_E = (float32_t)(-sin(st_GPSIMU.st_RefWGSRAD.f64_Longitude) * f64_dX + cos(st_GPSIMU.st_RefWGSRAD.f64_Longitude) * f64_dY);
    st_GPSIMU.f32_N = (float32_t)(-sin(st_GPSIMU.st_RefWGSRAD.f64_Latitude) * cos(st_GPSIMU.st_RefWGSRAD.f64_Longitude) * f64_dX -
                                  sin(st_GPSIMU.st_RefWGSRAD.f64_Latitude) * sin(st_GPSIMU.st_RefWGSRAD.f64_Longitude) * f64_dY +
                                  cos(st_GPSIMU.st_RefWGSRAD.f64_Latitude) * f64_dZ);
    st_GPSIMU.f32_U = (float32_t)(cos(st_GPSIMU.st_RefWGSRAD.f64_Latitude) * cos(st_GPSIMU.st_RefWGSRAD.f64_Longitude) * f64_dX +
                                  cos(st_GPSIMU.st_RefWGSRAD.f64_Latitude) * sin(st_GPSIMU.st_RefWGSRAD.f64_Longitude) * f64_dY +
                                  sin(st_GPSIMU.st_RefWGSRAD.f64_Latitude) * f64_dZ);
}

float32_t ENU2NED(float32_t f32_Heading)
{
    if (-180 < f32_Heading && f32_Heading < -90)
    {
        return -270 - f32_Heading;
    }
    else
    {
        return 90 - f32_Heading;
    }
}

void XY2EN_LiDAR(std::vector<std::vector<float32_t>> &vf_LiDARObject, std::vector<std::vector<float32_t>> &vf_LiDARObjectYMinMax, std::vector<int32_t> &vs_LiDARClass, LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning)
{
    float32_t f32_Theta = M_PI / 2 - st_GPSIMU.f32_Heading_rad; 
    float32_t f32_X, f32_Y, f32_YMin, f32_YMax;
    int32_t s32_Num = 0;
    for (int32_t s32_i = 0; s32_i < vf_LiDARObject.size(); s32_i++)
    {
        f32_X = st_GPSIMU.f32_E +
                cos(f32_Theta) * (vf_LiDARObject[s32_i][0] + c_LiDARPOSX) - sin(f32_Theta) * (vf_LiDARObject[s32_i][1] + c_LIDARPOSY);
        f32_Y = st_GPSIMU.f32_N +
                sin(f32_Theta) * (vf_LiDARObject[s32_i][0] + c_LiDARPOSX) + cos(f32_Theta) * (vf_LiDARObject[s32_i][1] + c_LIDARPOSY);
        f32_YMin = st_GPSIMU.f32_N +
                sin(f32_Theta) * (vf_LiDARObject[s32_i][0] + c_LiDARPOSX) + cos(f32_Theta) * (vf_LiDARObjectYMinMax[s32_i][0] + c_LIDARPOSY);
        f32_YMax = st_GPSIMU.f32_N +
                sin(f32_Theta) * (vf_LiDARObject[s32_i][0] + c_LiDARPOSX) + cos(f32_Theta) * (vf_LiDARObjectYMinMax[s32_i][1] + c_LIDARPOSY);
        if (st_Planning.st_Mission.s_SelectName != "U_TURN" && st_Planning.st_Mission.s_SelectName != "AVOID_SMALL" && st_Planning.st_Mission.s_SelectName != "AVOID_LARGE" && st_Planning.st_Mission.s_SelectName != "NARROW" && st_Planning.st_Mission.s_SelectName != "TUNNEL")
        {
            if (InOrOut(st_Planning, f32_X, f32_Y))
            {
                st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
                st_LiDAR.st_EN[s32_Num] = {f32_X, f32_Y};
                st_LiDAR.st_ENyMin[s32_Num] = {f32_X, f32_YMin};
                st_LiDAR.st_ENyMax[s32_Num] = {f32_X, f32_YMax};
                st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
                st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
                st_LiDAR.ars32_Classes[s32_Num] = vs_LiDARClass[s32_i];
                s32_Num++;
            }
            else if (st_Planning.st_Mission.s_SelectName == "PARKING")
            {
                st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
                st_LiDAR.st_EN[s32_Num] = {f32_X, f32_Y};
                st_LiDAR.st_ENyMin[s32_Num] = {f32_X, f32_YMin};
                st_LiDAR.st_ENyMax[s32_Num] = {f32_X, f32_YMax};
                st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
                st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
                st_LiDAR.ars32_Classes[s32_Num] = vs_LiDARClass[s32_i];
                s32_Num++;
            }
        }
        else if (st_Planning.st_Mission.s_SelectName == "AVOID_SMALL")
        {
            int32_t f32_ObjectNearIdx = FindNear_LiDAR(st_Planning, vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1], st_Planning.s32_RefPathEndIdx);
            Point2D_t st_ObjectNearPoint = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
            float32_t f32_ObjectDis = Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y ,f32_ObjectNearIdx, st_ObjectNearPoint);

            if(f32_ObjectDis < 1.4)
            {
                st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
                st_LiDAR.st_EN[s32_Num] = {f32_X, f32_Y};
                st_LiDAR.st_ENyMin[s32_Num] = {f32_X, f32_YMin};
                st_LiDAR.st_ENyMax[s32_Num] = {f32_X, f32_YMax};
                st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
                st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
                st_LiDAR.ars32_Classes[s32_Num] = vs_LiDARClass[s32_i];
                s32_Num++;
            }
        }
        else if (st_Planning.st_Mission.s_SelectName == "AVOID_LARGE")
        {
            int32_t f32_ObjectNearIdx = FindNear_LiDAR(st_Planning, vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1], st_Planning.s32_RefPathEndIdx);
            Point2D_t st_ObjectNearPoint = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
            float32_t f32_ObjectDis = Normalize2D(st_Planning.st_RefPath.arf32_X, st_Planning.st_RefPath.arf32_Y ,f32_ObjectNearIdx, st_ObjectNearPoint);

            Point2D_t PathVector = UnitVector(st_Planning.st_RefPath.arf32_X[f32_ObjectNearIdx], st_Planning.st_RefPath.arf32_Y[f32_ObjectNearIdx], 
                                        st_Planning.st_RefPath.arf32_X[f32_ObjectNearIdx + 1], st_Planning.st_RefPath.arf32_Y[f32_ObjectNearIdx + 1]);
            Point2D_t ObjVector = UnitVector(st_Planning.st_RefPath.arf32_X[f32_ObjectNearIdx], st_Planning.st_RefPath.arf32_Y[f32_ObjectNearIdx], 
                                        vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]);
            bool ObjCrossProduct = CrossProduct(PathVector, ObjVector);

            // if(f32_ObjectDis < 3.4)
            // {
            //     st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
            //     st_LiDAR.st_EN[s32_Num] = {f32_X, f32_Y};
            //     st_LiDAR.st_ENyMin[s32_Num] = {f32_X, f32_YMin};
            //     st_LiDAR.st_ENyMax[s32_Num] = {f32_X, f32_YMax};
            //     st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
            //     st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
            //     st_LiDAR.ars32_Classes[s32_Num] = vs_LiDARClass[s32_i];
            //     s32_Num++;
            // }

            if(ObjCrossProduct == 1 && f32_ObjectDis < 1.7)
            {
                st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
                st_LiDAR.st_EN[s32_Num] = {f32_X, f32_Y};
                st_LiDAR.st_ENyMin[s32_Num] = {f32_X, f32_YMin};
                st_LiDAR.st_ENyMax[s32_Num] = {f32_X, f32_YMax};
                st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
                st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
                st_LiDAR.ars32_Classes[s32_Num] = vs_LiDARClass[s32_i];
                s32_Num++;
            }
            else if(ObjCrossProduct == 0 && f32_ObjectDis < 5.1)
            {
                st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
                st_LiDAR.st_EN[s32_Num] = {f32_X, f32_Y};
                st_LiDAR.st_ENyMin[s32_Num] = {f32_X, f32_YMin};
                st_LiDAR.st_ENyMax[s32_Num] = {f32_X, f32_YMax};
                st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
                st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
                st_LiDAR.ars32_Classes[s32_Num] = vs_LiDARClass[s32_i];
                s32_Num++;
            }
        }
        else
        {
            st_LiDAR.st_XY[s32_Num] = {vf_LiDARObject[s32_i][0], vf_LiDARObject[s32_i][1]};
            st_LiDAR.arf32_Size[s32_Num] = vf_LiDARObject[s32_i][2];
            st_LiDAR.arf32_Distance[s32_Num] = vf_LiDARObject[s32_i][3];
            s32_Num++;
        }
    }
    st_LiDAR.s32_ObjectNum = s32_Num;
}

void XY2EN_Fusion(std::vector<std::vector<float32_t>> &vf_FusionObject, std::vector<int32_t> &vs_FusionClass, Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning)
{
    float32_t f32_Theta = M_PI / 2  - st_GPSIMU.f32_Heading_rad; // Heading = yaw (North 0, Est +90)
    float32_t f32_X, f32_Y;
    int32_t s32_Num = 0;
    for (int32_t s32_i = 0; s32_i < vf_FusionObject.size(); s32_i++)
    {
        f32_X = st_GPSIMU.f32_E +
                cos(f32_Theta) * (vf_FusionObject[s32_i][0] + c_LiDARPOSX) - sin(f32_Theta) * (vf_FusionObject[s32_i][1] + c_LIDARPOSY);
        f32_Y = st_GPSIMU.f32_N +
                sin(f32_Theta) * (vf_FusionObject[s32_i][0] + c_LiDARPOSX) + cos(f32_Theta) * (vf_FusionObject[s32_i][1] + c_LIDARPOSY);
        if (InOrOut(st_Planning, f32_X, f32_Y))
        {
            st_Fusion.st_XY[s32_Num] = {vf_FusionObject[s32_i][0], vf_FusionObject[s32_i][1]};
            st_Fusion.st_EN[s32_Num] = {f32_X, f32_Y};
            st_Fusion.arf32_Size[s32_Num] = vf_FusionObject[s32_i][2];
            st_Fusion.arf32_Distance[s32_Num] = vf_FusionObject[s32_i][3];
            st_Fusion.ars32_Classes[s32_Num] = vs_FusionClass[s32_i];
            s32_Num++;
        }
    }
    st_Fusion.s32_ObjectNum = s32_Num;
}

bool InOrOut(Planning_t &st_Planning, float32_t f32_PointX, float32_t f32_PointY)
{
    int32_t s32_MeetNum = 0;
    float32_t f32_X, f32_NextX, f32_Y, f32_NextY, f32_Value;
    Path_t st_Path;

    // OutBoundary Meet Point Num
    for (int32_t s32_i = 0; s32_i < 5; s32_i++)
    {
        switch (s32_i)
        {
        case 0:
            st_Path = st_Planning.st_OutBoundary;
            break;
        case 1:
            st_Path = st_Planning.st_InBoundary1;
            break;
        case 2:
            st_Path = st_Planning.st_InBoundary2;
            break;
        case 3:
            st_Path = st_Planning.st_InBoundary3;
            break;
        case 4:
            st_Path = st_Planning.st_InBoundary3;
            break;
        }
        for (int32_t s32_j = 0; s32_j < st_Path.s32_PathNum; s32_j++)
        {
            if (st_Path.s32_PathNum == 0)
                break;
            f32_X = st_Path.arf32_E[s32_j];
            f32_NextX = st_Path.arf32_E[(s32_j + 1) % st_Path.s32_PathNum];

            f32_Y = st_Path.arf32_N[s32_j];
            f32_NextY = st_Path.arf32_N[(s32_j + 1) % st_Path.s32_PathNum];

            if ((f32_Y >= f32_PointY && f32_NextY < f32_PointY) || (f32_Y < f32_PointY && f32_NextY >= f32_PointY))
            {
                f32_Value = (f32_NextX + (f32_PointY - f32_NextY) / (f32_Y - f32_NextY) * (f32_X - f32_NextX));
                if (f32_Value < f32_PointX)
                {
                    s32_MeetNum += 1;
                }
            }
        }
    }

    if (s32_MeetNum % 2 == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void GetRadius(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_X3, float32_t f32_Y3, float32_t &Radius)
{
    float32_t d1 = (f32_X2 - f32_X1) / (f32_Y2 - f32_Y1);
    float32_t d2 = (f32_X3 - f32_X2) / (f32_Y3 - f32_Y2);
    float32_t cx = ((f32_Y3 - f32_Y1) + (f32_X2 + f32_X3) * d2 - (f32_X1 + f32_X2) * d1) / (2 * (d2 - d1));
    float32_t cy = -d1 * (cx - (f32_X1 + f32_X2) / 2) + (f32_Y1 + f32_Y2) / 2;
    Radius = sqrtf(powf(f32_X1 - cx, 2) + powf(f32_Y1 - cy, 2));
}

void GetRadius(float64_t f32_X1, float64_t f32_Y1, float64_t f32_X2, float64_t f32_Y2, float64_t f32_X3, float64_t f32_Y3, float64_t &Radius)
{
    float64_t d1 = (f32_X2 - f32_X1) / (f32_Y2 - f32_Y1);
    float64_t d2 = (f32_X3 - f32_X2) / (f32_Y3 - f32_Y2);
    float64_t cx = ((f32_Y3 - f32_Y1) + (f32_X2 + f32_X3) * d2 - (f32_X1 + f32_X2) * d1) / (2 * (d2 - d1));
    float64_t cy = -d1 * (cx - (f32_X1 + f32_X2) / 2) + (f32_Y1 + f32_Y2) / 2;
    Radius = sqrtf(powf(f32_X1 - cx, 2) + powf(f32_Y1 - cy, 2));
}

float32_t pi2pi(float32_t f32_Angle)
{
    if (f32_Angle > M_PI)
        f32_Angle -= 2 * M_PI;
    else if (f32_Angle < -M_PI)
        f32_Angle += 2 * M_PI;

    return f32_Angle;
}

float64_t pi2pi(float64_t f64_Angle)
{
    if (f64_Angle > M_PI)
        f64_Angle -= 2 * M_PI;
    else if (f64_Angle < -M_PI)
        f64_Angle += 2 * M_PI;

    return f64_Angle;
}

void GetModData(float32_t min, float32_t max, float32_t &data)
{
    if (data > max)
    {
        data = max;
    }
    else if (data < min)
    {
        data = min;
    }
}

void GetModData(float64_t min, float64_t max, float64_t &data)
{
    if (data > max)
    {
        data = max;
    }
    else if (data < min)
    {
        data = min;
    }
}

void GetModData(int32_t min, int32_t max, int32_t &data)
{
    if (data > max)
    {
        data = max;
    }
    else if (data < min)
    {
        data = min;
    }
}

bool InRange(float32_t min, float32_t max, float32_t data)
{
    if (min <= data && data <= max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool InRange(float64_t min, float64_t max, float64_t data)
{
    if (min <= data && data <= max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool InRange(int32_t min, int32_t max, int32_t data)
{
    if (min <= data && data <= max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void OverShootFilter(float32_t f32_Maxtick, float32_t f32_Pre_data, float32_t &f32_Target_data)
{
    float32_t f32_dsteer = f32_Target_data - f32_Pre_data;

    if ((f32_dsteer) >= f32_Maxtick)
    {
        f32_Target_data = f32_Pre_data + f32_Maxtick;
    }
    else if ((f32_dsteer) <= -f32_Maxtick)
    {
        f32_Target_data = f32_Pre_data - f32_Maxtick;
    }
}

void OverShootFilter(float64_t f64_Maxtick, float64_t f64_Pre_data, float64_t &f64_Target_data)
{
    float32_t f64_dsteer = f64_Target_data - f64_Pre_data;

    if (f64_dsteer >= f64_Maxtick)
    {
        f64_Target_data = f64_Pre_data + f64_Maxtick;
    }
    else if (f64_dsteer <= -f64_Maxtick)
    {
        f64_Target_data = f64_Pre_data - f64_Maxtick;
    }
}

void removeDuplicates(std::vector<std::vector<float>> &vec)
{
    std::set<std::vector<float>, VectorCompare> s(vec.begin(), vec.end());
    vec.assign(s.begin(), s.end());
}

bool isObtuse(float x0, float y0, float x1, float y1, float x2, float y2)
{
    Eigen::Vector2d A(x1 - x0, y1 - y0);
    Eigen::Vector2d B(x2 - x0, y2 - y0);
    Eigen::Vector2d C(x2 - x1, y2 - y1);
    Eigen::Vector2d D(x0 - x1, y0 - y1);
    Eigen::Vector2d E(x0 - x2, y0 - y2);
    Eigen::Vector2d F(x1 - x2, y1 - y2);

    float lengthA = A.norm();
    float lengthB = B.norm();
    float lengthC = C.norm();
    float lengthD = D.norm();
    float lengthE = E.norm();
    float lengthF = F.norm();

    // Check if any angle is obtuse (dot product < 0)
    if (A.dot(B) < lengthA * lengthB * -0.3 ||
        C.dot(D) < lengthC * lengthD * -0.3 ||
        E.dot(F) < lengthE * lengthF * -0.3)
    {
        return true;
    }
    return false;
}