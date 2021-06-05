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
        anchors.push_back(make_pair(189, Point(-0.052632, 1.000000, 0.052632)));
        anchors.push_back(make_pair(38, Point(1.00000, 0.00000, 1.00000)));
        anchors.push_back(make_pair(399, Point(1.00000, 0.00000, -1.00000)));
        anchors.push_back(make_pair(381, Point(-1.00000, 0.00000, -1.00000)));
        anchors.push_back(make_pair(0, Point(-1.00000, 0.00000, 1.00000)));
        mesh.SetConstraints(anchors);
    }
    else{
        cout << "Usage: ./test_plane ${Obj_path}" << endl;
    }
    SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());
    glutMainLoop();

    return 0;
}