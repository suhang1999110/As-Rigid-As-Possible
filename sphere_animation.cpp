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


void sphere_deform1(){
    auto north_pole = mesh.vList[32];
    auto sourth_pole = mesh.vList[37];
    auto now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    double t = (now.count() - start_time.count()) / 1000.0;
    double shift = sin(t);

    mesh.vList[32]->SetPosition(Vector3d(north_pole->Position().X(), north_pole->Position().Y(),1+shift));
    mesh.vList[37]->SetPosition(Vector3d(north_pole->Position().X(), north_pole->Position().Y(),-1-shift));
    mesh.SetConstraints({32,37});
    mesh.Deform(1, UNIFORM);
    glutPostRedisplay();
}


int main(int argc, char **argv){
    string path = string(PROJECT_DIR) + "data/sphere_small.obj";
    glutInit(&argc, argv);
    InitGL();
    InitMenu();
    mesh.LoadObjFile(path.c_str());
    InitAnchorList();
    SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());

    glutIdleFunc(sphere_deform1);
    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());

    glutMainLoop();
}