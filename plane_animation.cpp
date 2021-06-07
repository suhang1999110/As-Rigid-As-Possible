//
// Created by suhang on 2021/6/7.
//

#include "visualizer.hpp"
#include <math.h>
#include <random>
#include <chrono>
#include "config.h"
#include <string>

std::chrono::milliseconds start_time;



void plane_deform1(){
    auto now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    double t = (now.count() - start_time.count()) / 1000.0;
    auto x = -0.052632 + sin(t);
    auto y = 1;
    auto z = 0.052632 + cos(t);
    mesh.vList[189]->SetPosition(Vector3d(x, y, z));
    mesh.SetConstraints({189});
    mesh.Deform(1, UNIFORM);
    glutPostRedisplay();
}

void plane_deform2(){
    auto now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    double t = (now.count() - start_time.count()) / 1000.0;
    int idx1 = 126, idx2 = 273;
    auto x1 = mesh.vList[idx1]->Position().X();
    auto y1 = sin(t);
    auto z1 = mesh.vList[idx1]->Position().Z();
    mesh.vList[idx1]->SetPosition(Vector3d(x1, y1, z1));
    auto x2 = mesh.vList[idx2]->Position().X();
    auto y2 = -sin(t);
    auto z2 = mesh.vList[idx2]->Position().Z();
    mesh.vList[idx2]->SetPosition(Vector3d(x2, y2, z2));
    mesh.SetConstraints({idx1,idx2});
    mesh.Deform(1, UNIFORM);
    glutPostRedisplay();
}


int main(int argc, char **argv){
    string path = string(PROJECT_DIR) + "data/plane.obj";
    glutInit(&argc, argv);
    InitGL();
    InitMenu();
    mesh.LoadObjFile(path.c_str());
    InitAnchorList();
    SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());

    glutIdleFunc(plane_deform1);
    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());

    glutMainLoop();
}