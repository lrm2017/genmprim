#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <vector>
#include <QVector>

#include "json.hpp"

#include "directCollocationSolver.h"
#include "utils.hpp"
#include "surfacegraph.h"

#include "project_path.h"

using namespace std;
//using namespace Eigen;


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    MovementPrimitives mps;
    Movement3DPrimitives mps3d;

    plot = ui->widget;
    plot->setShowLegend(false);

    double resolution = 0.02;

    connect(ui->generate, &QPushButton::clicked, this, [this, resolution]{
        Eigen::MatrixXi point(2,3);
        point << 1,2,3,4,4,5;
        generateMP("mp", point, resolution);
        plot->setShowLegend(false);
        plot->show();
    });

    connect(ui->read, &QPushButton::clicked, this, [this, &mps]{
//        plot->clear();
        readPrimitive(std::string(PROJECT_PATH)+"/mp.json", mps);
        plot->setShowLegend(false);
        plot->show();
    });

//    Eigen::MatrixXi point(2,3);
//    point << 1,2,3,4,4,5;
//    generateMP("mp", point, resolution);
//    plot->setShowLegend(false);
//    plot->show();

//    readPrimitive(std::string(PROJECT_PATH)+"/mp.json", mps);
//    showPrimitive(mps);
//    plot->setShowLegend(false);
//    plot->show();

//    generate3DMP("3d_mp", resolution);

    // 显示三维运动基元
    readPrimitive(std::string(PROJECT_PATH)+"/3d_mp.json", mps3d);
    show3DPrimitive(mps3d);


//    plot->setMaximumHeight(2);
//    plot->setMinimumHeight(-2);
//    plot->setAxisEqual(true);

}

template <class T>
void MainWindow::readPrimitive(const std::string& filename, T& mps){
    auto [j, flag] = readJson(filename);
    if(!flag){
        cout << "json文件读取失败，请检查路径是否正确" << endl;
        return;
    }
    mps = j;
}

void MainWindow::showPrimitive(MovementPrimitives& mps){
    int car=0;
    vector<vector<float>> color = {
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f}
    };

    cout << "运动基元分辨率：" << mps.resolution << "\t";
    cout << "运动基元总数目：" << mps.totalNumberOfPrimitives << "\t";
    cout << "运动基元角度分辨率:" << 360.0/mps.numberOfAngles << endl;
    for( auto perAngle: mps.mps){
        for (auto prim : perAngle) {
            cout << "primID:" << prim.primID << "\t";
            cout << "start angle:" << 360.0*prim.startAngle/mps.numberOfAngles << "\t";
            Eigen::RowVectorXi pose(3);
            pose << prim.endPose.x, prim.endPose.y, prim.endPose.theta;
            Eigen::RowVectorXf a = pose.cast<float>() * mps.resolution;
            a(2) = 360*a(2)/mps.numberOfAngles/mps.resolution;
            cout << "末端姿态" << a << endl;

            QCurve3D *curve = new QCurve3D; // 必须用指针，且需要初始化
            auto px = prim.primitive[0];    // x
            auto py = prim.primitive[1];    // y
            for (int x = 0; x < px.size(); ++x) {
                curve->addData(px[x], py[x],car);
            }
            QColor c;
            c.setRgbF(color[car][0],color[car][1],color[car][2],color[car][3]);
            curve->setColor(c);
            car++;
            car = car%color.size();
            curve->setLineWidth(2);
            plot->addCurve(curve);
        }
    }

}

void MainWindow::show3DPrimitive(Movement3DPrimitives& mps){
    int car=0;
    vector<vector<float>> color = {
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f}
    };

    cout << "运动基元分辨率：" << mps.resolution << "\t";
    cout << "运动基元总数目：" << mps.totalNumberOfPrimitives << "\t";
    cout << "运动基元角度分辨率:" << 360.0/mps.numberOfAngles << endl;
    for( auto perAngle: mps.mps){
        for (auto prim : perAngle) {
            for (auto slope : prim) {
                cout << "primID:" << slope.primID << "\t";
                cout << "start angle:" << 360.0*slope.startAngle/mps.numberOfAngles << "\t";
                Eigen::RowVectorXi pose(4);
                pose << slope.endPose.x, slope.endPose.y, slope.endPose.z, slope.endPose.theta;
                Eigen::RowVectorXf a = pose.cast<float>() * mps.resolution;
                a(3) = 360*a(3)/mps.numberOfAngles/mps.resolution;
                cout << "末端姿态" << a << endl;

                QCurve3D *curve = new QCurve3D; // 必须用指针，且需要初始化
                auto px = slope.primitive[0];    // x
                auto py = slope.primitive[1];    // y
                auto pz = slope.primitive[2];    // z
                for (int x = 0; x < px.size(); ++x) {
                    curve->addData(px[x], py[x], pz[x]);
                }
                QColor c;
                c.setRgbF(color[car][0],color[car][1],color[car][2],color[car][3]);
                curve->setColor(c);
                car++;
                car = car%color.size();
                curve->setLineWidth(2);
                plot->addCurve(curve);
            }
        }
    }
}

/**
 * @brief
 * @param filename
 * @param point
 * @param resolution
 */
void MainWindow::generateMP(std::string filename, Eigen::Matrix2Xi point, double resolution){
    using namespace Eigen ;

    MovementPrimitives mps;
    mps.resolution = resolution;


    State start, goal;
    start.state = {0, 0, 0, 0};

    const int N = 20; //  采样数量
    Settings set = {N, 1, 1, "mumps"};

    int numberofangles = 16;    //  360/16 = 22.5;
    int numberprimspreangle = 4;   // 每个角度有多少个运动基元

    mps.numberOfAngles = numberofangles;
    mps.totalNumberOfPrimitives = numberofangles*numberprimspreangle;

    std::vector<Vector4i> basemprimendpts0_c;    //
    basemprimendpts0_c.emplace_back(1,0,0,0);
    basemprimendpts0_c.emplace_back(8,0,0,0);
//    basemprimendpts0_c.emplace_back(-1,0,0,0);   // 倒退
    basemprimendpts0_c.emplace_back(8,1,1,0);
    basemprimendpts0_c.emplace_back(8,-1,-1,0);

    vector<Vector4i> basemprimendpts45_c;    //
    basemprimendpts45_c.emplace_back(1,1,0,0);
    basemprimendpts45_c.emplace_back(6,6,0,0);
//    basemprimendpts45_c.emplace_back(-1,-1,0,0);   // 倒退
    basemprimendpts45_c.emplace_back(5,7,1,0);
    basemprimendpts45_c.emplace_back(7,5,-1,0);

    vector<Vector4i> basemprimendpts22p5_c;
    basemprimendpts22p5_c.emplace_back(2, 1, 0, 0);
    basemprimendpts22p5_c.emplace_back(6, 3, 0, 0);
//    basemprimendpts22p5_c.emplace_back(-2,-1,0, 0);
    basemprimendpts22p5_c.emplace_back(5, 4, 1, 0);
    basemprimendpts22p5_c.emplace_back(7, 2, -1,0);

    vector<vector<float>> color = {
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f}
    };
    int car=0;
//    vector<Eigen::ArrayXXf> quadMps;    // 第一个象限运动基元集合
    //
    for (int i = 0; i < numberofangles/4; ++i) {  // 有多少个角度 ，只求解第一象限的

        car = 0;
        vector<perPrimitive> vecAng;
        for (int permind = 0; permind < numberprimspreangle; ++permind) { // 每个角度有多少个运动基元

            perPrimitive perPrimitive;
            perPrimitive.primID = i*numberprimspreangle + permind;
            perPrimitive.startAngle = i;

            double cur_angle = i*2*M_PI/numberofangles;
            int cur_angle_int = round(i*36000/numberofangles);  //

            Vector4i basemprimendpts;
            double angle;
            //
            if( cur_angle_int%9000 == 0){
                basemprimendpts = basemprimendpts0_c[permind];
                angle = cur_angle;
            }
            else if( cur_angle_int%4500 == 0){
                basemprimendpts = basemprimendpts45_c[permind];
                angle = cur_angle - 45*M_PI/180;
            }
            else if( (cur_angle_int-7875)%9000 == 0){

            }
            else if( (cur_angle_int-6750)%9000 == 0){
                basemprimendpts = basemprimendpts22p5_c[permind];
                basemprimendpts(0) = basemprimendpts22p5_c[permind](1); // 交换x、y
                basemprimendpts(1) = basemprimendpts22p5_c[permind](0);
                basemprimendpts(2) = -basemprimendpts22p5_c[permind](2);
                angle = cur_angle - 67.5*M_PI/180;
            }
            else if( (cur_angle_int-5625)%9000 == 0){

            }
            else if( (cur_angle_int-3375)%9000 == 0){

            }
            else if( (cur_angle_int-2250)%9000 == 0){
                basemprimendpts = basemprimendpts22p5_c[permind];
                angle = cur_angle - 22.5*M_PI/180;
                cout << "22p5" << endl;
            }
            else if( (cur_angle_int-1125)%9000 == 0){

            }
            else {
                cout << "错误：无效的角度精度, angle = " << cur_angle_int << endl;
                return;
            }

            int endx_c = round( basemprimendpts(0)* cos(angle)- basemprimendpts(1)* sin(angle));
            int endy_c = round( basemprimendpts(0)* sin(angle) + basemprimendpts(1)* cos(angle));
            int endtheta_c = (i+basemprimendpts(2))%numberofangles;

            Vector3i endpose_c;
            endpose_c << endx_c, endy_c, endtheta_c;

            perPrimitive.endPose = {endx_c, endy_c, endtheta_c};

            start.state = {0,0,cur_angle,0};
            goal.state = { endpose_c(0)*resolution, endpose_c(1)*resolution,
                           endtheta_c*2*M_PI/numberofangles,0 };

            directCollocationSolver solver(start, goal);    // 设置两端姿态
            solver.setProblemColloc(set);     // 设置
            bool isOk = solver.solveCollocation();
            DM sol_state;
            if(isOk)  sol_state = solver.getSolCollocation(solver.X);   // 获取迭代过程状态
            else {
                cout << "起始姿态：" << start.state  << endl;
                cout << "目标姿态：" << goal.state << endl;
                return;
            }
            perPrimitive.length = static_cast<double>(solver.getSolCollocation(solver.Sf));  //
            cout << "路径长度：" << perPrimitive.length << endl;

            cout << "state:\n" << sol_state << endl;
            size_t rows = sol_state.size1();
            size_t cols = sol_state.size2();
            vector<float> vector_x = static_cast< vector<float> >(sol_state);
            Eigen::ArrayXXf path = Eigen::Map<Eigen::ArrayXXf>(vector_x.data(), rows, cols);    // x y theta curve
//            quadMps.emplace_back(path); // 记录第一象限运动基元

            Eigen::VectorXf px = path.row(0); // x坐标轴
            Eigen::VectorXf py = path.row(1); //y 坐标轴
            Eigen::VectorXf pt = path.row(2); // theta
            Eigen::VectorXf pk = path.row(3);   // curve

            perPrimitive.primitive.emplace_back(px.data(),px.data()+ px.size());    //
            perPrimitive.primitive.emplace_back(py.data(),py.data()+py.size());
            perPrimitive.primitive.emplace_back(pt.data(),pt.data()+pt.size());
            perPrimitive.primitive.emplace_back(pk.data(),pk.data()+pt.size());

            vecAng.emplace_back(perPrimitive); // 保存一条运动基元的信息
        }
        mps.mps.emplace_back(vecAng);   // 相同起始角度的运动基元集合
    }
    cout << "第一象限的运动基元数量有：" << mps.mps.size() << endl;
    for (int i = 0; i < 3; ++i) {   // 将运动基元从第一象限分别旋转变换到第二、三、四象限
        double beta = (i+1)*M_PI/2;
        car = 0;
        for (int j = 0; j < numberofangles/4; ++j) {  // 第一象限的所有运动基元
            vector<perPrimitive> vecAng;
            for( int k = 0; k < numberprimspreangle; ++k){
                perPrimitive primitive = mps.mps[j][k];    //
                Eigen::MatrixXf path(primitive.primitive[0].size(),2);
                path.col(0) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(primitive.primitive[0].data(), primitive.primitive[0].size());
                path.col(1) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(primitive.primitive[1].data(), primitive.primitive[1].size());

//            Eigen::ArrayXXf primitive = quadMps[j]; // 2*N;
                Eigen::Matrix<float, 2, 2> rotate;  // 旋转矩阵
                rotate << cos(beta), sin(beta),
                        -sin(beta), cos(beta);
                Eigen::ArrayXXf r_prim = path*rotate;  // 2*N
                Eigen::VectorXf rpx = r_prim.col(0);
                Eigen::VectorXf rpy = r_prim.col(1);
                Eigen::VectorXf rpt = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(primitive.primitive[2].data(), primitive.primitive[2].size());
                Eigen::VectorXf ang(rpt.size());
                ang.setConstant(beta);
                rpt += ang;   // 航向方向
                primitive.primitive[0] = vector<float>(rpx.data(),rpx.data()+rpx.size());
                primitive.primitive[1] = vector<float>(rpy.data(),rpy.data()+rpy.size());
                primitive.primitive[2] = vector<float>(rpt.data(),rpt.data()+rpt.size());
                primitive.startAngle += (i+1)*numberofangles/4;
                Eigen::VectorXf endPose_int(2);
                endPose_int << primitive.endPose.x, primitive.endPose.y;
                Eigen::VectorXf ans = endPose_int.transpose()*rotate;
                primitive.endPose.x = ans(0);
                primitive.endPose.y = ans(1);
                primitive.endPose.theta += (i+1)*numberofangles/4;
                primitive.endPose.theta %= numberofangles;
                primitive.primID = mps.mps.size();
                vecAng.emplace_back(primitive);
            }
            mps.mps.emplace_back(vecAng);
        }
    }
    json j;
    j = mps;
    writeJson(std::string(PROJECT_PATH)+"/"+filename+".json", j);
    cout << "完成运动基元的生成" << endl;

}

/**
 * 生成三维运动基元
 * @param filename  文件名
 * @param resolution 分辨率
 */
void MainWindow::generate3DMP(std::string filename, double resolution){
    using namespace Eigen ;

    Movement3DPrimitives mps;
    mps.resolution = resolution;
    mps.start_slope = -15/180.0 * M_PI;
    double max_slope = 20/180.0 * M_PI;
    mps.res_slope = 5/180.0 * M_PI;
    mps.numberOfSlopes = (max_slope-mps.start_slope)/mps.res_slope;

    State start, goal;
    start.state = {0, 0, 0, 0};

    const int N = 20; //  采样数量
    Settings set = {N, 1, 1, "mumps"};

    int numberofangles = 16;    //  360/16 = 22.5;
    int numberprimspreangle = 4;   // 每个角度有多少个运动基元

    mps.numberOfAngles = numberofangles;
    mps.totalNumberOfPrimitives = numberofangles*numberprimspreangle;

    std::vector<Vector4i> basemprimendpts0_c;    //
    basemprimendpts0_c.emplace_back(1,0,0,0);
    basemprimendpts0_c.emplace_back(8,0,0,0);
//    basemprimendpts0_c.emplace_back(-1,0,0,0);   // 倒退
    basemprimendpts0_c.emplace_back(8,1,1,0);
    basemprimendpts0_c.emplace_back(8,-1,-1,0);

    vector<Vector4i> basemprimendpts45_c;    //
    basemprimendpts45_c.emplace_back(1,1,0,0);
    basemprimendpts45_c.emplace_back(6,6,0,0);
//    basemprimendpts45_c.emplace_back(-1,-1,0,0);   // 倒退
    basemprimendpts45_c.emplace_back(5,7,1,0);
    basemprimendpts45_c.emplace_back(7,5,-1,0);

    vector<Vector4i> basemprimendpts22p5_c;
    basemprimendpts22p5_c.emplace_back(2, 1, 0, 0);
    basemprimendpts22p5_c.emplace_back(6, 3, 0, 0);
//    basemprimendpts22p5_c.emplace_back(-2,-1,0, 0);
    basemprimendpts22p5_c.emplace_back(5, 4, 1, 0);
    basemprimendpts22p5_c.emplace_back(7, 2, -1,0);

    vector<vector<float>> color = {
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.00f, 0.00f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f},
            {0.f, 0.83f, 0.14f, 0.14f},
            {0.f, 1.00f, 0.54f, 0.00f},
            {0.f, 0.47f, 0.25f, 0.80f},
            {0.f,0.25f, 0.80f, 0.54f}
    };
    int car=0;
//    vector<Eigen::ArrayXXf> quadMps;    // 第一个象限运动基元集合
    //
//    vector<vector<vector<perPrimitive3d>>> base_0c;
    for (int i = 0; i < numberofangles/4; ++i) {  // 有多少个角度 ，只求解第一象限的
        car = 0;
        vector<vector<perPrimitive3d>> vecSlope;    // 一个角度有多少个运动基元*多个坡度
        for (int permind = 0; permind < numberprimspreangle; ++permind) { // 每个角度有多少个运动基元

            perPrimitive3d perPrimitive;
            perPrimitive.primID = i*numberprimspreangle + permind;
            perPrimitive.startAngle = i;

            double cur_angle = i*2*M_PI/numberofangles;
            int cur_angle_int = round(i*36000/numberofangles);  //

            Vector4i basemprimendpts;
            double angle;
            //
            if( cur_angle_int%9000 == 0){
                basemprimendpts = basemprimendpts0_c[permind];
                angle = cur_angle;
            }
            else if( cur_angle_int%4500 == 0){
                basemprimendpts = basemprimendpts45_c[permind];
                angle = cur_angle - 45*M_PI/180;
            }
            else if( (cur_angle_int-7875)%9000 == 0){

            }
            else if( (cur_angle_int-6750)%9000 == 0){
                basemprimendpts = basemprimendpts22p5_c[permind];
                basemprimendpts(0) = basemprimendpts22p5_c[permind](1); // 交换x、y
                basemprimendpts(1) = basemprimendpts22p5_c[permind](0);
                basemprimendpts(2) = -basemprimendpts22p5_c[permind](2);
                angle = cur_angle - 67.5*M_PI/180;
            }
            else if( (cur_angle_int-5625)%9000 == 0){

            }
            else if( (cur_angle_int-3375)%9000 == 0){

            }
            else if( (cur_angle_int-2250)%9000 == 0){
                basemprimendpts = basemprimendpts22p5_c[permind];
                angle = cur_angle - 22.5*M_PI/180;
                cout << "22p5" << endl;
            }
            else if( (cur_angle_int-1125)%9000 == 0){

            }
            else {
                cout << "错误：无效的角度精度, angle = " << cur_angle_int << endl;
                return;
            }

            int endx_c = round( basemprimendpts(0)* cos(angle)- basemprimendpts(1)* sin(angle));
            int endy_c = round( basemprimendpts(0)* sin(angle) + basemprimendpts(1)* cos(angle));
            int endtheta_c = (i+basemprimendpts(2))%numberofangles;

            Vector3i endpose_c;
            endpose_c << endx_c, endy_c, endtheta_c;

            perPrimitive.endPose = {endx_c, endy_c, 0, endtheta_c};

            start.state = {0,0,cur_angle,0};
            goal.state = { endpose_c(0)*resolution, endpose_c(1)*resolution,
                           endtheta_c*2*M_PI/numberofangles,0 };

            directCollocationSolver solver(start, goal);    // 设置两端姿态
            solver.setProblemColloc(set);     // 设置
            bool isOk = solver.solveCollocation();
            DM sol_state;
            if(isOk)  sol_state = solver.getSolCollocation(solver.X);   // 获取迭代过程状态
            else {
                cout << "起始姿态：" << start.state  << endl;
                cout << "目标姿态：" << goal.state << endl;
                return;
            }
            perPrimitive.length = static_cast<double>(solver.getSolCollocation(solver.Sf));  //
            cout << "路径长度：" << perPrimitive.length << endl;

            cout << "state:\n" << sol_state << endl;
            size_t rows = sol_state.size1();
            size_t cols = sol_state.size2();
            vector<float> vector_x = static_cast< vector<float> >(sol_state);
            Eigen::ArrayXXf path = Eigen::Map<Eigen::ArrayXXf>(vector_x.data(), rows, cols);    // x y theta curve
//            quadMps.emplace_back(path); // 记录第一象限运动基元

            Eigen::VectorXf px = path.row(0); // x坐标轴
            Eigen::VectorXf py = path.row(1); //y 坐标轴
            Eigen::VectorXf pz = Eigen::VectorXf::Zero(py.size());
            Eigen::VectorXf pt = path.row(2); // theta
            Eigen::VectorXf pk = path.row(3);   // curve
            perPrimitive.primitive.emplace_back(px.data(),px.data()+ px.size());
            perPrimitive.primitive.emplace_back(py.data(),py.data()+ py.size());
            perPrimitive.primitive.emplace_back(pz.data(),pz.data()+ pz.size());
            perPrimitive.primitive.emplace_back(pt.data(),pt.data()+ pt.size());
            perPrimitive.primitive.emplace_back(pk.data(),pk.data()+ pk.size());

            Eigen::MatrixXf pf(px.size(), 3);
            pf << px, py, pz ;  // 旋转变换不需要齐次坐标

            vector<perPrimitive3d> perSlope;    // 一条运动基元对应的多个坡度的
            for(double i=mps.start_slope; i<=mps.maxSlope(); i += mps.res_slope){
                auto prim = perPrimitive;

                Eigen::Matrix<float, 3, 3> rotate;  // 旋转矩阵
                rotate << cos(i),0, -sin(i),
                       0, 1, 0,
                        sin(i),0, cos(i);
                Eigen::MatrixXf mat = pf*rotate;
//                decltype(perPrimitive.primitive) prim;  // 自动推导变量类型
                Eigen::VectorXf mx = mat.col(0);
                Eigen::VectorXf my = mat.col(1);
                Eigen::VectorXf mz = mat.col(2);
                prim.primitive[0] = vector<float>(mx.data(), mx.data()+mx.size());
                prim.primitive[1] = vector<float>(my.data(), my.data()+my.size());
                prim.primitive[2] = vector<float>(mz.data(), mz.data()+mz.size());
                perSlope.emplace_back(prim);
            }
            vecSlope.emplace_back(perSlope);
//
//            perPrimitive.primitive.emplace_back(px.data(),px.data()+ px.size());    //
//            perPrimitive.primitive.emplace_back(py.data(),py.data()+py.size());
//            perPrimitive.primitive.emplace_back(pz.data(),pz.data()+pz.size());     // z = 0;
//            perPrimitive.primitive.emplace_back(pt.data(),pt.data()+pt.size());
//            perPrimitive.primitive.emplace_back(pk.data(),pk.data()+pt.size());
//
//            vecAng.emplace_back(perPrimitive); // 保存一条运动基元的信息
        }
        mps.mps.emplace_back(vecSlope);   // 相同起始角度的运动基元集合  角度
    }
    cout << "第一象限的运动基元数量有：" << mps.mps.size() << endl;
    for (int i = 0; i < 3; ++i) {   // 将运动基元从第一象限分别旋转变换到第二、三、四象限
        double beta = (i+1)*M_PI/2;
        car = 0;
        for (int j = 0; j < numberofangles/4; ++j) {  // 第一象限的所有运动基元
            vector<vector<perPrimitive3d>> vecAng;
            for( int k = 0; k < numberprimspreangle; ++k){  //
                vector<perPrimitive3d> angSlope;
                for(int s = 0; s < mps.numberOfSlopes; s++){    // 坡度
                    perPrimitive3d primitive = mps.mps[j][k][s];    //
                    Eigen::MatrixXf path(primitive.primitive[0].size(),3);
                    path.col(0) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(
                            primitive.primitive[0].data(), primitive.primitive[0].size());  // x
                    path.col(1) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(
                            primitive.primitive[1].data(), primitive.primitive[1].size());  // y
                    path.col(2) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(
                            primitive.primitive[2].data(), primitive.primitive[2].size());  // z

//            Eigen::ArrayXXf primitive = quadMps[j]; // 2*N;
                    Eigen::Matrix<float, 3, 3> rotate;  // 绕z轴旋转beta
                    rotate << cos(beta), sin(beta), 0,
                            -sin(beta), cos(beta), 0,
                            0, 0, 1;
                    Eigen::ArrayXXf r_prim = path*rotate;  // 2*N
                    Eigen::VectorXf rpx = r_prim.col(0);
                    Eigen::VectorXf rpy = r_prim.col(1);
                    Eigen::VectorXf rpz = r_prim.col(2); // z=0
                    Eigen::VectorXf rpt = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(primitive.primitive[3].data(), primitive.primitive[3].size());
                    Eigen::VectorXf ang(rpt.size());
                    ang.setConstant(beta);
                    rpt += ang;   // 航向方向
                    primitive.primitive[0] = vector<float>(rpx.data(),rpx.data()+rpx.size());
                    primitive.primitive[1] = vector<float>(rpy.data(),rpy.data()+rpy.size());
                    primitive.primitive[2] = vector<float>(rpz.data(),rpz.data()+rpz.size());
                    primitive.primitive[3] = vector<float>(rpt.data(),rpt.data()+rpt.size());
                    primitive.startAngle += (i+1)*numberofangles/4;
                    Eigen::VectorXf endPose_int(3);
                    endPose_int << primitive.endPose.x, primitive.endPose.y, primitive.endPose.z;
                    Eigen::VectorXf ans = endPose_int.transpose()*rotate;
                    primitive.endPose.x = ans(0);
                    primitive.endPose.y = ans(1);
                    primitive.endPose.z = ans(2);
                    primitive.endPose.theta += (i+1)*numberofangles/4;
                    primitive.endPose.theta %= numberofangles;
                    primitive.primID = mps.mps.size();
                    angSlope.emplace_back(primitive);
                }
                vecAng.emplace_back(angSlope);
            }
            mps.mps.emplace_back(vecAng);
        }
    }
    json j;
    j = mps;
    writeJson(std::string(PROJECT_PATH)+"/"+filename+".json", j);
    cout << "完成运动基元的生成" << endl;
}


MainWindow::~MainWindow()
{
    delete ui;
}

