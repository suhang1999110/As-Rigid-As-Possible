//
// Created by suhang on 2021/6/6.
//

#include "visualizer.hpp"

int main(int argc, char **argv){
    glutInit(&argc, argv);
    InitGL();
    InitMenu();
    if(argc == 2){
        mesh.LoadObjFile(argv[1]);
        AnchorPair anchors;
        anchors.push_back(make_pair(32, Point(0.000000, 0.000000, 1.20000)));
        anchors.push_back(make_pair(37, Point(0.000000, 0.000000, -1.200000)));
        mesh.SetConstraints(anchors);
    }
    else{
        cout << "Usage: ./test_sphere ${Obj_path}" << endl;
    }
    SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());
    glutMainLoop();

    return 0;
}