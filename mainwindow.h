#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Eigen/Eigen>

#include "QPlot3D.h"

#include "utils.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    union MPrim{ MovementPrimitives prim; Movement3DPrimitives prim3d;};

    template<class T>
    void readPrimitive(const std::string& filename, T& mps );
    void generateMP(std::string filename, Eigen::Matrix2Xi point, double resolution);
    void generate3DMP(std::string filename, double resolution);

    void showPrimitive(MovementPrimitives& mps);
    void show3DPrimitive(Movement3DPrimitives& mps);

private:
    Ui::MainWindow *ui;
    QPlot3D *plot;

};
#endif // MAINWINDOW_H
