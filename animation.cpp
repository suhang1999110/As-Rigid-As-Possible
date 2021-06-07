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


void plane_deform2(){
    auto now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    double t = (now.count() - start_time.count()) / 1000.0;
    auto x = mesh.vList[189]->Position().X();
    auto y = sin(t);
    auto z = mesh.vList[189]->Position().Z();
    mesh.vList[189]->SetPosition(Vector3d(x, y, z));
    mesh.SetConstraints({189});
    mesh.Deform(1, UNIFORM);
    glutPostRedisplay();
}

void plane_deform3(){
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
    string path = string(PROJECT_DIR) + "data/plane.obj";
    cout<<path<<endl;
    glutInit(&argc, argv);
    InitGL();
    InitMenu();
    if(argc == 2){
        mesh.LoadObjFile(path.c_str());
        InitAnchorList();
    }
    else{
        cout << "Usage: ./main ${Obj_path}" << endl;
    }
    SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());

    glutIdleFunc(plane_deform2);
    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());

    glutMainLoop();
}