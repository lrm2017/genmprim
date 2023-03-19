#pragma once
#ifndef UTILS_HPP
#define UTILS_HPP
#include "json.hpp"
#include <vector>
#include <fstream>
#include <fstream>
#include <Eigen/Eigen>
using json = nlohmann::json;
using namespace Eigen;
using namespace std;

#define MAX(X,Y) (((X)>(Y)) ? (X) : (Y))
#define MIN(X,Y) (((X)<(Y)) ? (X) : (Y))

//
#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))
#define DISCXY2CONT(X, CELLSIZE) ((X)*(CELLSIZE) + (CELLSIZE)/2.0)

//#define M_PI 3.141592653589793238462643383279502884
#define UNKNOWN_COST 1000000

// 将 vector<vector<>> 转换为 matrix
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> convert_vvd_to_matrix(vector<vector<T> >& vvd) {

    const std::size_t n_rows = vvd.size();
    const std::size_t n_cols = vvd.at(0).size();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result(n_rows, n_cols);
    result.row(0) = VectorXf::Map(&vvd[0][0], n_cols);

    for (std::size_t i = 1; i < n_rows; i++) {
        if (n_cols != vvd.at(i).size()) {
            char buffer[200];
            snprintf(buffer, 200,
                     "vvd[%ld] size (%ld) does not match vvd[0] size (%ld)",
                     i, vvd.at(i).size(), n_cols);
            string err_mesg(buffer);
            throw std::invalid_argument(err_mesg);
        }
        result.row(i) =Eigen::Matrix<T, 1, Eigen::Dynamic>::Map(&vvd[i][0], n_cols);
    }
    return result;
}

// 将 matrix 转换为 vector<vector<>>
template <typename T>
vector<vector<T>> convert_matrix_to_vv(Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic> mat){
    const std::size_t n_rows = mat.rows();
//    const std::size_t n_cols = mat.cols();
    vector<vector<T>> vv;//(n_rows, vector<T>(n_cols));
    for (std::size_t i = 0; i < n_rows; ++i) {
        //
        Eigen::Matrix<T, 1, Eigen::Dynamic> v=mat.row(i); // 不能直接用mat.row(i)的地址，因为是按列来保存的
        vv.push_back(vector<T>(v.data(), v.data()+v.size()));
    }
    return vv;
}

template <class T>
struct stateXYT  {
public:
    stateXYT():x(0), y(0), theta(0){};
    stateXYT(T x_, T y_, T theta_):
            x(x_), y(y_), theta(theta_){};
    void setCoor(std::pair<T,T> coor){
        x = coor.first;
        y = coor.second;
    }
    bool operator==(const stateXYT& s) const{
        return x==s.x && y==s.y && theta==s.theta;
    }
    friend void to_json(json& j, const stateXYT& t);
    friend void from_json(const json& j, stateXYT& t);

public:
    T x;
    T y;
    T theta; // 运动基元id
};

struct perPrimitive{
    int primID;
    int startAngle;
    stateXYT<int> endPose;    // x y t
    vector<vector<float>> primitive;    // 一条运动基元  4*N,
    double length;  // 路径长度
    double cost;    // 路径代价
    friend void to_json(json& j, const perPrimitive& t);
    friend void from_json(const json& j, perPrimitive& t);
};

class MovementPrimitives{
public:
    double resolution;  // 分辨率
    int numberOfAngles; // 360分为多少个角度
    int totalNumberOfPrimitives;    // 总共多少条运动基元
//    vector<MatrixXf> mps;   // 运动基元  N*3

    vector<vector<perPrimitive>> mps;   // numberofangle * peranglenumber

    friend void to_json(json& j, const MovementPrimitives& t);
    friend void from_json(const json& j, MovementPrimitives& t);
};

/***********************************************
 ********* 三维运动基元
 ***********************************************/
template <class T>
struct stateXYZT  {
public:
    stateXYZT():x(0), y(0), z(0), theta(0){};
    stateXYZT(T x_, T y_, T z_, T theta_):
            x(x_), y(y_), z(z_), theta(theta_){};
    void setCoor(T x_, T y_, T z_){
        x = x_;
        y = y_;
        z = z_;
    }
    bool operator==(const stateXYZT& s) const{
        return x==s.x && y==s.y && z==s.z && theta==s.theta;
    }
    friend void to_json(json& j, const stateXYZT& t);
    friend void from_json(const json& j, stateXYZT& t);

public:
    T x;
    T y;
    T z;
    T theta; // 运动基元id
};

struct perPrimitive3d{
    int primID;
    int startAngle;
    stateXYZT<int> endPose;    // x y z t
    vector<vector<float>> primitive;    // 一条运动基元  4*N,
    double length;  // 路径长度
    double cost;    // 路径代价
    friend void to_json(json& j, const perPrimitive3d& t);
    friend void from_json(const json& j, perPrimitive3d& t);
};

class Movement3DPrimitives{
public:
    double resolution;  // 分辨率
    double res_slope;   // 坡度的分辨率
    double start_slope; // 起始值 下坡到上坡 斜坡
    int numberOfSlopes; // 有多少个坡度
    int numberOfAngles; // 360分为多少个角度
    int totalNumberOfPrimitives;    // 总共多少条运动基元

    vector<vector<vector<perPrimitive3d>>> mps;   // numberOfSlopes*numberOfAngles*perAngleNumber

    int getIntSlope(double slope){
        double sl = MAX( start_slope+res_slope*numberOfSlopes, MIN(start_slope,slope) );
        return MAX(int(sl/res_slope), numberOfSlopes);
    }
    double getDoubleSlope(int slope){
        return slope*res_slope+start_slope;
    }
    double maxSlope(){return numberOfSlopes*res_slope+start_slope;};



    friend void to_json(json& j, const Movement3DPrimitives& t);
    friend void from_json(const json& j, Movement3DPrimitives& t);
};


/**
 * @brief readJson
 * @param path
 * @return json and readfile flag
 */
pair<json, bool> readJson(string path);

/**
 * @brief writeJson
 * @param file name
 * @param json
 */
void writeJson(string path, json j);

/**
 * @brief transform  类似python的  a = [data for i in b function()]
 * @param x:data
 * @param fn:function
 * @return
 */
Eigen::VectorXd transform(Eigen::VectorXd &x, std::function<double(double)> fn);


#endif
