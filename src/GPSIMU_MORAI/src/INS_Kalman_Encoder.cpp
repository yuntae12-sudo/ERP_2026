#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <sstream>
#include <istream>
#include <ros/ros.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <sensor_msgs/Imu.h>
#include <Total_msgs/GNSSInfo.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>
// morai 버전
#include <Total_msgs/GPSMessage.h>
#include <Total_msgs/EgoVehicleStatus.h>

using namespace std;
using namespace Eigen;

Total_msgs::GNSSInfo INS;

class SubandPub
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;

    Matrix<float, 3, 1> V;
    Matrix<float, 3, 3> Cbn;
    Matrix<float, 3, 1> f_ned;
    Matrix<float, 3, 1> V_ned;
    Matrix<float, 3, 3> c1;
    Matrix<float, 3, 3> c2;
    Matrix<float, 3, 1> c3;
    Matrix<float, 1, 3> c31;
    Matrix<float, 1, 3> c32;
    Matrix<float, 1, 3> c33;
    Matrix<float, 1, 3> c34;
    Matrix<float, 3, 1> c35;
    Matrix<float, 3, 1> m_7;
    Matrix<float, 3, 1> acc;
    Matrix<float, 3, 1> Vt;
    Matrix<float, 3, 1> Vt2;
    Matrix<float, 3, 1> Pt;
    Matrix<float, 3, 1> Pt2;
    Matrix<float, 3, 3> av;
    Matrix<float, 3, 1> g;
    Matrix<float, 3, 3> av_exp;
    Matrix<float, 3, 3> av_2;

    Matrix<float, 1, 9> X;
    Matrix<float, 15, 1> dX;
    Matrix<float, 15, 15> R;
    Matrix<float, 15, 1> dX0;
    Matrix<float, 15, 15> Q;
    Matrix<float, 15, 15> H;
    Matrix<float, 15, 15> K;
    Matrix<float, 15, 15> P;
    Matrix<float, 15, 15> F;
    Matrix<float, 15, 15> A;

    Matrix<float, 1, 3> w_ibb;
    Matrix<float, 1, 3> w_enn;
    Matrix<float, 1, 3> w_ien;
    Matrix<float, 1, 3> w_inn;

    Matrix<float, 3, 3> F_pp;
    Matrix<float, 3, 3> F_pv;
    Matrix<float, 3, 3> F_vp;
    Matrix<float, 3, 3> F_vv;
    Matrix<float, 3, 3> F_vphi;
    Matrix<float, 3, 3> F_phip;
    Matrix<float, 3, 3> F_phiv;
    Matrix<float, 3, 3> F_phiphi;
    Matrix<float, 3, 3> minCbn;

    Matrix<float, 4, 1> q;
    Matrix<float, 1, 4> v_quat;

    Vector3f euler_angles;

    double INSTime, lastINST;
    double GPSTime, lastGPST;
    double lat = 0, lon = 0, alt = 0, h = 0;
    double gps_lat = 1, gps_lon = 1, gps_alt = 1;
    double cos_lat, sin_lat, tan_lat;
    double cur_E_prev, cur_N_prev, cur_E_aft, cur_N_aft;
    double lat_origin, lon_origin, alt_origin;                                                         // origin LLA(rad)
    double ecc = 0.0818192, R0 = 6378137.0, Ome = 7.2921151467e-5, gravity = -9.81, ecc_2 = ecc * ecc; // earth value
    double wgs84_f = 1 / 298.257223563, wgs84_e2 = 2 * wgs84_f - pow(wgs84_f, 2);                      // wgs84 model

    float av_x, av_y, av_z;
    float la_x, la_y, la_z;
    float qu_x, qu_y, qu_z, qu_w;

    float roll, pitch, yaw;
    float q_roll, q_pitch, q_yaw;
    float rho_n, rho_e, rho_d;
    float f_n, f_e, f_d;
    float Rm, Rt, Rmm, Rtt;
    float v_e = 0, v_n = 0, v_u = 0;
    float gps_v_e = 0, gps_v_n = 0, gps_v_u = 0;
    float cur_E, cur_N, cur_U;
    float prev_E = 0, prev_N = 0, prev_U = 0;
    float d_E = 0, d_N = 0, d_U = 0;
    float degree = M_PI / 180.0, radian = 180.0 / M_PI; // degree -> rad / rad -> degree
    float dt, dt_gps, vel;
    float prev_roll, prev_pitch, prev_yaw, prev_lat, prev_lon, prev_alt, prev_v_e, prev_v_n, prev_v_u;

    int setting_yaw = 0;
    int vel_count = 0;
    float vel_mean = 0;
    float vel_sum = 0;
    float error = 0;
    float error_sum = 0;
    float error_mean = 0;

    bool flag1 = 0, flag2 = 0, IMU_flag = 0, GPS_flag = 0, Ref_flag = 0;

public:
    SubandPub()
    {
        initialsetting();
        sub1 = nh.subscribe<Total_msgs::GPSMessage>("/gps", 1, &SubandPub::gpsCallback, this);
        sub2 = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &SubandPub::imuCallback, this);
        sub3 = nh.subscribe<Total_msgs::EgoVehicleStatus>("/Ego_topic", 1, &SubandPub::EncorderCallback, this);

        pub = nh.advertise<Total_msgs::GNSSInfo>("/GNSScom", 1);
    }

    void initialsetting()
    {
        dX(0, 0) = -6.97664e-09;
        dX(1, 0) = 7.15927e-09;
        dX(2, 0) = 0.33608;
        dX(3, 0) = -0.00283069;
        dX(4, 0) = -0.00158075;
        dX(5, 0) = 0.000746894;
        dX(6, 0) = 0.000721;
        dX(7, 0) = -0.00100877;
        dX(8, 0) = 0.000739909;
        dX(9, 0) = -0.231594;
        dX(10, 0) = 0.152261;
        dX(11, 0) = 0.08364;
        dX(12, 0) = 3.3118e-06;
        dX(13, 0) = 2.10972e-05;
        dX(14, 0) = -6.86601e-06;
        dX0 = dX;

        Q.setZero();
        Q(0, 0) = 1000;
        Q(1, 1) = 1000;
        Q(2, 2) = 1000;
        Q(3, 3) = 1000000;
        Q(4, 4) = 1000000;
        Q(5, 5) = 1000000;
        Q(6, 6) = 10000000;
        Q(7, 7) = 10000000;
        Q(8, 8) = 10000000;
        Q(9, 9) = 1;
        Q(10, 10) = 1;
        Q(11, 11) = 1;
        Q(12, 12) = 100;
        Q(13, 13) = 100;
        Q(14, 14) = 100;

        R.setIdentity();
        R(0, 0) = 0.000000001;
        R(1, 1) = 0.000000001;
        R(2, 2) = 0.000000000001;
        R(3, 3) = 0.01;
        R(4, 4) = 0.01;
        R(5, 5) = 0.01;
        R(6, 6) = 0.00001;
        R(7, 7) = 0.00001;
        R(8, 8) = 0.00001;

        H.setIdentity();

        P.setIdentity();
        P = P * 0.1;

        F.setZero();

        V_ned.setZero();
    }
    void IMUSetting()
    {
        cout << "Setting yaw : ";
        cin >> setting_yaw;

        X(0, 6) = q_roll;
        X(0, 7) = q_pitch;
        X(0, 8) = setting_yaw;

        yaw = setting_yaw;
    }
    void ENUconverter(double &lat, double &lon, double &alt, float &cur_E, float &cur_N, float &cur_U)
    {
        double lat_point, lon_point, alt_point; // find LLA(rad)
        lat_point = lat * degree;
        lon_point = lon * degree;
        alt_point = alt;

        double chi = sqrt(1 - wgs84_e2 * pow(sin(lat_point), 2));
        double q = (R0 / chi + alt_point) * cos(lat_point);

        double find_x = q * cos(lon_point);
        double find_y = q * sin(lon_point);
        double find_z = ((R0 * (1 - wgs84_e2) / chi) + alt_point) * sin(lat_point);

        double ref_chi = sqrt(1 - wgs84_e2 * pow(sin(lat_origin), 2));
        double ref_q = (R0 / ref_chi + alt_origin) * cos(lat_origin);
        double ref_x = ref_q * cos(lon_origin);
        double ref_y = ref_q * sin(lon_origin);
        double ref_z = ((R0 * (1 - wgs84_e2) / ref_chi) + alt_origin) * sin(lat_origin);

        double dx = find_x - ref_x;
        double dy = find_y - ref_y;
        double dz = find_z - ref_z;

        cur_E = -sin(lon_origin) * dx + cos(lon_origin) * dy;
        cur_N = -sin(lat_origin) * cos(lon_origin) * dx - sin(lat_origin) * sin(lon_origin) * dy + cos(lat_origin) * dz;
        cur_U = (cos(lat_origin) * cos(lon_origin) * dx + cos(lat_origin) * sin(lon_origin) * dy + sin(lat_origin) * dz);
    }

    void INS_ERROR_MODEL()
    {
        lat = X(0, 0);
        lon = X(0, 1);
        alt = X(0, 2);
        v_n = X(0, 3);
        v_e = X(0, 4);
        v_u = -X(0, 5);
        roll = X(0, 6);
        pitch = X(0, 7);
        yaw = X(0, 8);

        // earth model
        Rm = R0 * (1 - ecc_2) / pow((1 - ecc_2 * pow((sin(lat * degree)), 2)), 1.5);                                                     // 위도(lat)에 따라 북-남 방향으로의 원경거리
        Rt = R0 / (1 - ecc_2 * pow(pow((sin(lat * degree)), 2), 0.5));                                                                   // 위도(lat)에 따라 동-서 방향으로의 원경거리
        Rmm = (3 * R0 * (1 - ecc_2) * ecc_2 * sin(lat * degree)) * cos(lat * degree) / pow((1 - ecc_2 * ((sin(lat * degree)), 2)), 2.5); // 위도(lat)에 따라 북-남 방향의 곡률 반경
        Rtt = R0 * ecc_2 * sin(lat * degree) * cos(lat * degree) / pow((1 - ecc_2 * pow((sin(lat * degree)), 2)), 1.5);                  // 위도(lat)에 따라 동-서 방향의 곡률 반경

        av.setZero();
        g.setZero();

        w_ibb << av_x, av_y, av_z;

        // ECEF에 대한 nav frame의 angular rate
        w_enn << v_e / (Rt + h), -v_n / (Rm + h), -v_e * tan(lat * degree) / (Rt + h);
        rho_n = w_enn(0);
        rho_e = w_enn(1);
        rho_d = w_enn(2);

        // 지구 자전 각속도
        w_ien << Ome * cos(lat * degree), 0, -Ome * sin(lat * degree);

        // 위치와 속력을 사용하여 구한 각속도
        w_inn << w_ien(0, 0) + w_enn(0, 0), w_ien(0, 1) + w_enn(0, 1), w_ien(0, 2) + w_enn(0, 2);

        // 회전행렬
        Cbn << cos(pitch * degree) * cos(yaw * degree),
            cos(yaw * degree) * sin(pitch * degree) * sin(roll * degree) -
                cos(roll * degree) * sin(yaw * degree),
            cos(roll * degree) * cos(yaw * degree) * sin(pitch * degree) +
                sin(roll * degree) * sin(yaw * degree),
            cos(pitch * degree) * sin(yaw * degree),
            cos(roll * degree) * cos(yaw * degree) +
                sin(pitch * degree) * sin(roll * degree) * sin(yaw * degree),
            cos(roll * degree) * sin(pitch * degree) * sin(yaw * degree) -
                cos(yaw * degree) * sin(roll * degree),
            -sin(pitch * degree),
            cos(pitch * degree) * sin(roll * degree),
            cos(pitch * degree) * cos(roll * degree); // body to ned DCM  eul2DCM_bn

        c1 = skew(w_ibb(0, 0) - dX(12, 0), w_ibb(0, 1) - dX(13, 0), w_ibb(0, 2) - dX(14, 0));
        c2 = skew(w_inn(0, 0), w_inn(0, 1), w_inn(0, 2));

        acc << la_x, la_y, la_z;

        f_ned = Cbn * acc; // Cosine direction matrix @=matrix x matrix body to ned accel
        f_n = f_ned(0, 0); // n accel
        f_e = f_ned(1, 0); // e accel
        f_d = f_ned(2, 0); // d accel

        Cbn = Cbn + (Cbn * c1 - c2 * Cbn) * dt; // mechanization DCM differential equation
        DCM2eul_bn(Cbn);

        c31 << w_ien(0, 0) * 2, w_ien(0, 1) * 2, w_ien(0, 2) * 2;
        c32 << w_enn(0, 0), w_enn(0, 1), w_enn(0, 2);
        c33 = c31 + c32;
        c34 << c33(0, 1) * V_ned(2, 0) - c33(0, 2) * V_ned(1, 0),
            c33(0, 2) * V_ned(0, 0) - c33(0, 0) * V_ned(2, 0),
            c33(0, 1) * V_ned(1, 0) - c33(0, 1) * V_ned(0, 0);
        c35 = reshape(c34, 3, 1);
        c3 << la_x - dX(9, 0) - c35(0, 0), la_y - dX(10, 0) - c35(1, 0), la_z - dX(11, 0) - c35(2, 0);

        // 속력 업데이트
        V_ned << v_n, v_e, -v_u;
        g << 0, 0, gravity;
        m_7 = Cbn * c3 - c35;
        V = V_ned + (m_7 + g) * dt;

        v_n = V(0, 0);
        v_e = V(1, 0);
        v_u = -V(2, 0);

        // Position 업데이트

        lat = lat + (radian) * (v_n / (Rm + h)) * dt;
        lon = lon + (radian) * (v_e / ((Rt + h) * cos(lat * degree))) * dt;
        alt = alt + (v_u)*dt;
        h = h + (v_u)*dt;

        X << lat, lon, alt, v_n, v_e, -v_u, roll, pitch, yaw;
    }

    void INS_MECHANIZATION()
    {
        cos_lat = cos(lat * degree);
        sin_lat = sin(lat * degree);
        tan_lat = tan(lat * degree);

        float H1 = Rm + h;
        float H2 = Rt + h;

        F_pp << Rmm * rho_e / H1, 0, rho_e / H1,
            rho_n * (tan_lat - Rtt / H2) / cos_lat, 0, -rho_n / (cos_lat * H2),
            0, 0, 0;

        F_pv << 1 / H1, 0, 0,
            0, 1 / (cos_lat * H2), 0,
            0, 0, -1;

        F_vp << Rmm * rho_e * (-v_u) / H1 - (rho_n / (pow(cos_lat, 2)) + 2 * w_ien(0)) * v_e - rho_n * rho_d * Rtt, 0,
            rho_e * (-v_u) / (H1)-rho_n * rho_d,
            (2 * w_ien(0) + rho_n / (pow(cos_lat, 2)) + rho_d * Rtt / H2) * v_n - (rho_n * Rtt / H2 - 2 * w_ien(2)) * (-v_u), 0, rho_d * v_n / (H2)-rho_n * (-v_u) / (H2),
            (pow(rho_n, 2)) * Rtt + (pow(rho_e, 2)) * Rmm - 2 * w_ien(2) * v_e, 0, pow(rho_n, 2) + pow(rho_e, 2);

        F_vv << (-v_u) / H1, 2 * rho_d + 2 * w_ien(2), -rho_e,
            -2 * w_ien(2) - rho_d, (v_n * tan_lat - v_u) / (H2), 2 * w_ien(0) + rho_n,
            2 * rho_e, -2 * w_ien(0) - 2 * rho_n, 0;

        F_vphi << 0, -f_d, f_e,
            f_d, 0, -f_n,
            -f_e, f_n, 0;

        F_phip << w_ien(2) - rho_n * Rtt / H2, 0, -rho_n / H2,
            -rho_e * Rmm / H1, 0, -rho_e / H1,
            -w_ien(0) - rho_n / (pow(cos_lat, 2)) - rho_d * Rtt / H2, 0, -rho_d / H2;

        F_phiv << 0, 1 / H2, 0,
            -1 / H1, 0, 0,

            0, -tan_lat / H2, 0;

        F_phiphi << 0, w_ien(2) + rho_d, -rho_e,
            -w_ien(2) - rho_d, 0, w_ien(0) + rho_n,
            rho_e, -w_ien(0) - rho_n, 0;

        minCbn = (-1) * Cbn;

        F.setZero();

        for (int i = 0; i < 3; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                F(i, k) = F_pp(i, k);
                F(i, k + 3) = F_pv(i, k);
                F(i + 3, k) = F_vp(i, k);
                F(i + 3, k + 3) = F_vv(i, k);
                F(i + 3, k + 6) = F_vphi(i, k);
                F(i + 3, k + 9) = Cbn(i, k);
                F(i + 6, k) = F_phip(i, k);
                F(i + 6, k + 3) = F_phiv(i, k);
                F(i + 6, k + 6) = F_phiphi(i, k);
                F(i + 6, k + 12) = minCbn(i, k);
            }
        }
    }

    void Prediction()
    {
        Eigen::MatrixXf F2(15, 15);
        F2 = dt * F;
        A = F2.exp();
        dX = A * dX;
        P = A * P * A.transpose() + Q;
    }

    void Update()
    {
        Eigen::MatrixXf z(15, 1);

        K = P * (H.transpose()) * ((H * P * H.transpose() + R).inverse());

        z.setZero();

        z(0, 0) = X(0, 0) - gps_lat;
        z(1, 0) = X(0, 1) - gps_lon;
        z(2, 0) = X(0, 2) - gps_alt;
        z(3, 0) = X(0, 3) - gps_v_n;
        z(4, 0) = X(0, 4) - gps_v_e;
        z(5, 0) = X(0, 5) - gps_v_u;

        dX = dX + (K * (z - H * dX));
        P = P - K * H * P;

        X(0, 0) = X(0, 0) - dX(0, 0); // lla
        X(0, 1) = X(0, 1) - dX(1, 0);
        X(0, 2) = X(0, 2) - dX(2, 0);
        X(0, 3) = X(0, 3) - dX(3, 0); // V_ned
        X(0, 4) = X(0, 4) - dX(4, 0);
        X(0, 5) = X(0, 5) - dX(5, 0);
        X(0, 6) = X(0, 6) - dX(6, 0); // Roll Pitch Yaw
        X(0, 7) = X(0, 7) - dX(7, 0);
        X(0, 8) = X(0, 8) - dX(8, 0);

        dX = dX0;
    }

    Eigen::MatrixXf skew(float x, float y, float z)
    {
        Eigen::MatrixXf M(3, 3);
        M << 0, -z, y,
            z, 0, -x,
            -y, x, 0;

        return M;
    }

    void DCM2eul_bn(const Eigen::MatrixXf &matrix)
    {
        euler_angles[0] = atan2(matrix(2, 1), matrix(2, 2)) * 180.0 / M_PI;
        euler_angles[1] = asin(-matrix(2, 0)) * 180.0 / M_PI;
        euler_angles[2] = atan2(matrix(1, 0), matrix(0, 0)) * 180.0 / M_PI;
        roll = euler_angles[0];
        pitch = euler_angles[1];
        yaw = euler_angles[2];
    }

    Eigen::MatrixXf reshape(Eigen::MatrixXf m, int col, int row)
    {
        Eigen::MatrixXf M(col, row);
        M << m(0, 0), m(0, 1), m(0, 2);
        return M;
    }

    void EncorderCallback(const Total_msgs::EgoVehicleStatus::ConstPtr &msg_encorder)
    {
        vel_count++;

        vel = sqrt(pow(msg_encorder->velocity.x, 2) + pow(msg_encorder->velocity.y, 2));
        vel_sum += vel;
        vel_mean = vel_sum / vel_count;

        error = vel - vel_mean;
        error_sum += error;
        error_mean = error_sum / vel_count;

        v_quat << 0, vel, 0, 0;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_imu)
    {
        INSTime = msg_imu->header.stamp.toSec();

        if (flag1 == 0)
        {
            lastINST = INSTime;
            flag1 = 1;
            dt = 0.02;
        }
        else
        {
            dt = INSTime - lastINST;
            lastINST = INSTime;
        }

        if (GPS_flag == true)
        {

            av_x = msg_imu->angular_velocity.x;
            av_y = msg_imu->angular_velocity.y;
            av_z = (msg_imu->angular_velocity.z);

            la_x = msg_imu->linear_acceleration.x;
            la_y = msg_imu->linear_acceleration.y;
            la_z = (msg_imu->linear_acceleration.z);

            qu_x = msg_imu->orientation.x;
            qu_y = msg_imu->orientation.y;
            qu_z = msg_imu->orientation.z;
            qu_w = msg_imu->orientation.w;

            float t0 = 2 * (qu_w * qu_x + qu_y * qu_z);
            float t1 = 1 - 2 * (qu_x * qu_x + qu_y * qu_y);
            float t2 = sqrt(1 + 2 * (qu_w * qu_y - qu_z * qu_x));
            float t3 = sqrt(1 - 2 * (qu_w * qu_y - qu_z * qu_x));

            float norm_q = qu_w * qu_w + qu_x * qu_x + qu_y * qu_y + qu_z * qu_z;

            Matrix<float, 4, 1> q_inv;
            Matrix<float, 4, 1> result1;
            Matrix<float, 3, 1> result2;
            Matrix<float, 3, 1> result3;

            q_inv << qu_w / norm_q, -qu_x / norm_q, -qu_y / norm_q, -qu_z / norm_q;

            result1 = q_inv * v_quat * q;

            result2 << result1(1, 0), result1(2, 0), result1(3, 0);

            result3 << Cbn * result2;

            X(0, 3) = result3(0, 0);
            X(0, 4) = result3(1, 0);
            X(0, 5) = result3(2, 0);

            q_roll = atan2(t0, t1) * radian;
            q_pitch = (2 * atan2(t2, t3) - M_PI / 2) * radian;

            if (IMU_flag == false)
            {
                IMUSetting();
                IMU_flag = true;
            }

            INS_ERROR_MODEL();

            INS_MECHANIZATION();

            Prediction();

            ENUconverter(lat, lon, alt, cur_E, cur_N, cur_U); // 좌표계 전환

            cout << "------------------Kalman Filter After--------------------" << endl;
            cout << "lat: " << lat << "    "
                 << "lon: " << lon << "    "
                 << "alt: " << alt << endl;

            // cout << "cur_E: " << cur_E << "    "
            //      << "cur_N: " << cur_N << "    "
            //      << "cur_U: " << cur_U << endl;
            cout << "kalman yaw: " << yaw << endl;

            prev_yaw = yaw;
            prev_lat = lat;
            prev_lon = lon;
            prev_alt = h;

            INS.heading = SubandPub::yaw;
            INS.latitude = SubandPub::lat;
            INS.longitude = SubandPub::lon;
            INS.altitude = SubandPub::alt;

            // raw gps 쓸거면 이거 주석 풀면 됨
            // INS.latitude = SubandPub::gps_lat;
            // INS.longitude = SubandPub::gps_lon;
            // INS.altitude = SubandPub::gps_alt;

            pub.publish(INS);
        }
    }

    void gpsCallback(const Total_msgs::GPSMessage::ConstPtr &msg)
    {

        GPSTime = msg->header.stamp.toSec();

        if (flag2 == 0)
        {
            lastGPST = GPSTime;
            flag2 = 1;
            dt_gps = 0.1;
        }
        else
        {
            dt_gps = GPSTime - lastGPST;
            lastGPST = GPSTime;
        }
        if (Ref_flag == 0)
        {
            lat_origin = (msg->latitude) * degree;
            lon_origin = (msg->longitude) * degree;
            alt_origin = msg->altitude;
            Ref_flag = 1;
        }
        if (GPS_flag == false)
        {
            X(0, 0) = msg->latitude;
            X(0, 1) = msg->longitude;
            X(0, 2) = msg->altitude;
            X(0, 3) = 0;
            X(0, 4) = 0;
            X(0, 5) = 0;

            gps_lat = msg->latitude;
            gps_lon = msg->longitude;
            gps_alt = msg->altitude;

            lat = gps_lat;
            lon = gps_lon;
            alt = gps_alt;
            h = gps_alt;

            GPS_flag = true;
        }

        if (IMU_flag == true)
        {
            gps_lat = msg->latitude;
            gps_lon = msg->longitude;
            gps_alt = msg->altitude;

            lat = gps_lat;
            lon = gps_lon;
            alt = gps_alt;
            h = gps_alt;

            ENUconverter(lat, lon, alt, cur_E, cur_N, cur_U);

            d_E = cur_E - prev_E;
            d_N = cur_N - prev_N;
            d_U = cur_U - prev_U;

            gps_v_e = d_E / dt_gps;
            gps_v_n = d_N / dt_gps;
            gps_v_u = d_U / dt_gps;

            v_e = gps_v_e;
            v_n = gps_v_n;
            v_u = gps_v_u;

            prev_N = cur_N;
            prev_E = cur_E;
            prev_U = cur_U;

            Update();
        }
    }
};

int main(int argc, char **argv)
{
    cout << fixed;
    cout.precision(15);

    ros::init(argc, argv, "GPSIMU");
    SubandPub SaP;
    ros::spin();
}