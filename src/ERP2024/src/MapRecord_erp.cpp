#include <ros/ros.h>
#include <Total_msgs/GNSSInfo.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/package.h>

using namespace std;
using namespace ros;
using namespace Eigen;

float norm(vector<float> &&v) {
    float sum = 0;
    for (auto &i : v) {
        sum += i * i;
    }
    return sqrt(sum);
}

vector<float> operator-(vector<float> a, vector<float> b) {
    assert(a.size() == b.size());
    for(int i=0; i<a.size(); i++) 
        a[i] -= b[i];
    return a;
}

vector<float> operator*(const vector<vector<float>>& m, const vector<float>& v) {
    assert(m[0].size() == v.size());
    vector<float> res;
    for(int i=0; i<m.size(); i++) {
        float sum = 0;
        for(int j=0; j<m[i].size(); j++) {
            sum += m[i][j] * v[j];
        }
        res.push_back(sum);
    }
    return res;
}


vector<float> prv = {0,0};
vector<float> lla2ecef(vector<double> lla);

const double a = 6378137; // 지구 반지름
const double e = 8.1819190842622e-2; // 이심률
std::ofstream mapFile;
vector<vector<float>> ecef2enu_(3, vector<float>(3, 0));


// vector<double> lla_spawn = {37.450108576465752, 126.651460584022260, 37.362521166741701}; //기존
// vector<double> lla_spawn = {37.449792508288404, 126.651369320224958, 36.424845680205507};
// vector<double> lla_spawn = {37.450040050735531, 126.656974045291562, 43.063240310674004}; // 농구장
// vector<double> lla_spawn = {37.450744223631780, 126.651141985020885, 42.593919508232950}; // 구령대
vector<double> lla_spawn = {37.365585531666667, 126.725147028333339, 33.353000000000002}; // 시흥
		
vector<float> ecef_spawn(3);


void setup() {
    mapFile.open("/home/autonav/ERP_2024/src/Map/Siheung/Path/Tmp.txt");
    ecef_spawn = lla2ecef(lla_spawn);

    double lat = lla_spawn[0] * (M_PIl / 180.0);
    double lon = lla_spawn[1] * (M_PIl / 180.0);

    ecef2enu_[0][0] = -sin(lon);
    ecef2enu_[0][1] = cos(lon);
    ecef2enu_[0][2] = 0;

    ecef2enu_[1][0] = -sin(lat)*cos(lon);
    ecef2enu_[1][1] = -sin(lat)*sin(lon);
    ecef2enu_[1][2] = cos(lat);

    ecef2enu_[2][0] = cos(lat)*cos(lon);
    ecef2enu_[2][1] = cos(lat)*sin(lon);
    ecef2enu_[2][2] = sin(lat);
}

vector<float> lla2ecef(vector<double> lla) {
    vector<float> ret(3, 0);
    double lat_rad = lla[0] * (M_PIl / 180l);
    double lon_rad = lla[1] * (M_PIl / 180l);
    double alt = lla[2];

    double N = a / sqrt(1 - e * e * sin(lat_rad) * sin(lat_rad));

    ret[0] = (N + alt) * cos(lat_rad) * cos(lon_rad);
    ret[1] = (N + alt) * cos(lat_rad) * sin(lon_rad);
    ret[2] = ((1 - e * e) * N + alt) * sin(lat_rad);
    return ret;
}
vector<float> lla2enu(double latitude, double longitude, double altitude) {
    vector<float> ecef_pos(3);
    double lat_rad = latitude * (M_PIl / 180l);
    double lon_rad = longitude * (M_PIl / 180l);

    double N = a / sqrt(1 - e * e * sin(lat_rad) * sin(lat_rad));

    ecef_pos[0] = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    ecef_pos[1] = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    ecef_pos[2] = ((1 - e * e) * N + altitude) * sin(lat_rad);

    ecef_pos = ecef_pos - ecef_spawn;

    auto enu_pos = ecef2enu_ * ecef_pos;
    return { enu_pos[0], enu_pos[1] };
}

void GPSGPS(const Total_msgs::GNSSInfo::ConstPtr &GPS)
{

    vector<float> pos = lla2enu(GPS->latitude, GPS->longitude, GPS->altitude);
    float dis = norm(pos - prv);
    if(dis > 0.1) {   // 1
        mapFile << std::setprecision(8) << pos[0] << ' '
                << std::setprecision(8) << pos[1] << '\n';
        prv = pos;
    }
    cout << "pos[1] : " << pos[1] << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "read_gps_node");
    ros::NodeHandle nh;

	ros::Subscriber GPSINS = nh.subscribe<Total_msgs::GNSSInfo>("/GNSScom", 1, GPSGPS);


    setup();
    std::cout << "start to write file" << std::endl;
    ros::spin();
    std::cout << "finish to write file" << std::endl;

    mapFile.close();
    return 0;   
}
