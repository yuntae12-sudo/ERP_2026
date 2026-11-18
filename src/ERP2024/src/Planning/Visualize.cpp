#include <Planning/Planning.hpp>

visualization_msgs::MarkerArray marker_array;

void VisualizeLine(std::vector<float32_t> vf32_StartPoint,
                   std::vector<float32_t> vf32_EndPoint,
                   std::vector<float32_t> vf32_RGBA,
                   float32_t f32_Scale)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = std::rand();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.1);

    marker.pose.position.x = 0.0f;
    marker.pose.position.y = 0.0f;
    marker.pose.position.z = 0.0f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;

    point.x = vf32_StartPoint[0];
    point.y = vf32_StartPoint[1];
    marker.points.push_back(point);
    point.x = vf32_EndPoint[0];
    point.y = vf32_EndPoint[1];
    marker.points.push_back(point);

    marker.color.r = vf32_RGBA[0];
    marker.color.g = vf32_RGBA[1];
    marker.color.b = vf32_RGBA[2];
    marker.color.a = vf32_RGBA[3];

    marker.scale.x = f32_Scale;

    marker_array.markers.push_back(marker);
}

void VisualizePoint(std::vector<float32_t> vf32_Point,
                    std::vector<float32_t> vf32_RGBA,
                    float32_t f32_Scale)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = std::rand();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.1);

    marker.pose.position.x = vf32_Point[0];
    marker.pose.position.y = vf32_Point[1];
    marker.pose.position.z = 0.0f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;

    marker.color.r = vf32_RGBA[0];
    marker.color.g = vf32_RGBA[1];
    marker.color.b = vf32_RGBA[2];
    marker.color.a = vf32_RGBA[3];

    marker.scale.x = f32_Scale;
    marker.scale.y = f32_Scale;
    marker.scale.z = f32_Scale;

    marker_array.markers.push_back(marker);
}

void VisualizeList(std::vector<std::vector<float32_t>> vf32_List,
                   std::vector<float32_t> vf32_RGBA,
                   float32_t f32_Scale)
{
    if (vf32_List.size() == 0)
    {
        return;
    }

    for (int32_t s32_i = 0; s32_i < vf32_List.size(); s32_i++)
    {
        VisualizePoint(vf32_List[s32_i], vf32_RGBA, f32_Scale);
    }
}

void VisualizeLocalPath(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning)
{
    std::vector<float32_t> vf32_Point;

    if (st_Planning.st_LocalPath.s32_PathNum == 0)
    {
        cout << "empty" << endl;
        return;
    }

    for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i++)
    {
        vf32_Point.clear();
        // vf32_Point = {st_Planning.st_LocalPath.arf32_E[s32_i], st_Planning.st_LocalPath.arf32_N[s32_i]};
        vf32_Point = {st_Planning.st_LocalPath.arf32_N[s32_i], st_Planning.st_LocalPath.arf32_E[s32_i]};
        VisualizePoint(vf32_Point, vf32_RGBA, f32_Scale);
    }
}

void VisualizeLocalPath(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning,
                        GPSIMU_t &st_GPSIMU)
{
    std::vector<float32_t> vf32_Point;

    if (st_Planning.st_LocalPath.s32_PathNum == 0)
    {
        cout << "empty" << endl;
        return;
    }

    for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i += 10)
    {
        vf32_Point.clear();
        vf32_Point = {st_Planning.st_LocalPath.arf32_N[s32_i] - st_GPSIMU.f32_N, st_Planning.st_LocalPath.arf32_E[s32_i] - st_GPSIMU.f32_E};
        VisualizePoint(vf32_Point, vf32_RGBA, f32_Scale);
    }
}

void VisualizeLocalPath_Avoid(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning)
{
    std::vector<float32_t> vf32_Point;

    if (st_Planning.st_LocalPath.s32_PathNum == 0)
    {
        cout << "empty" << endl;
        return;
    }

    for (int32_t s32_i = 0; s32_i < st_Planning.st_LocalPath.s32_PathNum; s32_i += 10)
    {
        vf32_Point.clear();
        // vf32_Point = {st_Planning.st_LocalPath.arf32_E[s32_i], st_Planning.st_LocalPath.arf32_N[s32_i]};
        vf32_Point = {st_Planning.st_LocalPath.arf32_N[s32_i], st_Planning.st_LocalPath.arf32_E[s32_i]};
        VisualizePoint(vf32_Point, vf32_RGBA, f32_Scale);
    }
}

void VisualizeRefPath(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning,
                        GPSIMU_t &st_GPSIMU)
{
    std::vector<float32_t> vf32_Point;

    if (st_Planning.st_RefPath.s32_PathNum == 0)
    {
        cout << "empty" << endl;
        return;
    }

    for (int32_t s32_i = 0; s32_i < st_Planning.st_RefPath.s32_PathNum; s32_i += 10)
    // for (int32_t s32_i = 0; s32_i < 2000; s32_i += 10)
    {
        vf32_Point.clear();
        vf32_Point = {st_Planning.st_RefPath.arf32_X[s32_i], st_Planning.st_RefPath.arf32_Y[s32_i]};
        VisualizePoint(vf32_Point, vf32_RGBA, f32_Scale);
    }
}

void Visualize()
{
    st_Visualize.publish(marker_array);
    marker_array.markers.clear();
}
