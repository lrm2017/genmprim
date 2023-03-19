#include "utils.hpp"
#include "Eigen/Eigen"
#include "Eigen/Dense"

// 只解析int类型的
void to_json(json& j, const stateXYT<int>& t){
    j = json{
            {"x", t.x},
            {"y", t.y},
            {"theta", t.theta}
    };
}

void from_json(const json& j, stateXYT<int>& t){
    j.at("x").get_to(t.x);
    j.at("y").get_to(t.y);
    j.at("theta").get_to(t.theta);
}


/**
 * @brief to_json
 * @param json
 * @param 单条运动基元
 */
void to_json(json& j, const perPrimitive& t){
    j = json{
            {"primID", t.primID},
            {"startAngle", t.startAngle},
            {"endPose", t.endPose},
            {"primitive", t.primitive},
            {"length", t.length},
            {"cost", t.cost}
    };
}

/**
 * @brief from_json
 * @param json
 * @param 单条运动基元
 */
void from_json(const json& j, perPrimitive& t){
    j.at("primID").get_to(t.primID);
    j.at("startAngle").get_to(t.startAngle);
    j.at("endPose").get_to(t.endPose);
    j.at("primitive").get_to<vector<vector<float>>>(t.primitive);
    j.at("length").get_to(t.length);
    j.at("cost").get_to(t.cost);
}

/**
 * @brief to_json
 * @param json
 * @param 运动基元
 */
void to_json(json& j, const MovementPrimitives& t){
    j = json{
            {"resolution", t.resolution},
            {"numberOfAngles", t.numberOfAngles},
            {"totalNumberOfPrimitives",t.totalNumberOfPrimitives},
            {"mps", t.mps}
    };
}

/**
 * @brief from_json
 * @param json
 * @param 运动基元
 */
void from_json(const json& j, MovementPrimitives& t){
    j.at("resolution").get_to(t.resolution);
    j.at("numberOfAngles").get_to(t.numberOfAngles);
    j.at("totalNumberOfPrimitives").get_to(t.totalNumberOfPrimitives);
    j.at("mps").get_to(t.mps);
}

/*********************************************************
 ************** 三维运动基元 *******************************
 *********************************************************/

void to_json(json& j, const stateXYZT<int>& t){
    j = json{
            {"x", t.x},
            {"y", t.y},
            {"z", t.z},
            {"theta", t.theta}
    };
}
void from_json(const json& j, stateXYZT<int>& t){
    j.at("x").get_to(t.x);
    j.at("y").get_to(t.y);
    j.at("z").get_to(t.z);
    j.at("theta").get_to(t.theta);
}

void to_json(json& j, const perPrimitive3d& t){
    j = json{
            {"primID", t.primID},
            {"startAngle", t.startAngle},
            {"endPose", t.endPose},
            {"primitive", t.primitive},
            {"length", t.length},
            {"cost", t.cost}
    };
}

void from_json(const json& j, perPrimitive3d& t){
    j.at("primID").get_to(t.primID);
    j.at("startAngle").get_to(t.startAngle);
    j.at("endPose").get_to(t.endPose);
    j.at("primitive").get_to<vector<vector<float>>>(t.primitive);
    j.at("length").get_to(t.length);
    j.at("cost").get_to(t.cost);
}

void to_json(json& j, const Movement3DPrimitives& t){
    j = json{
            {"resolution", t.resolution},
            {"numberOfAngles", t.numberOfAngles},
            {"res_slope", t.res_slope},
            {"start_slope", t.start_slope},
            {"numberOfSlopes", t.numberOfSlopes},
            {"totalNumberOfPrimitives",t.totalNumberOfPrimitives},
            {"mps", t.mps}
    };
}
void from_json(const json& j, Movement3DPrimitives& t){
    j.at("resolution").get_to(t.resolution);
    j.at("numberOfAngles").get_to(t.numberOfAngles);
    j.at("res_slope").get_to(t.res_slope);
    j.at("start_slope").get_to(t.start_slope);
    j.at("numberOfSlopes").get_to(t.numberOfSlopes);
    j.at("totalNumberOfPrimitives").get_to(t.totalNumberOfPrimitives);
    j.at("mps").get_to(t.mps);
}

/**
 * @brief readJson
 * @param file name
 * @return json
 */
pair<json, bool> readJson(string path){
    json j;
    std::ifstream file(path);
    if (!file.is_open()) {
        file.clear();
        return {j,false};
    }
    file >>  j;
    return {j, true};
}

/**
 * @brief writeJson
 * @param file name
 * @param json
 */
void writeJson(string path, json j){
    std::ofstream file(path,
    std::ios::trunc | std::ios::out | std::ios::in);
    file <<j;
}

/**
 * @brief transform  类似python的  a = [data for i in b function()]
 * @param x:data
 * @param fn:function
 * @return
 */
Eigen::VectorXd transform(Eigen::VectorXd &x, std::function<double(double)> fn){
    Eigen::VectorXd ans(x.size());
    for (int i = 0; i < x.size(); ++i) {
        ans(i) = fn(x(i));
    }
    return ans;
}

