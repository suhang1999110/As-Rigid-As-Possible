#include "visualizer.hpp"


// main function
int main(int argc, char **argv) {
    glutInit(&argc, argv);
    InitGL();
    InitMenu();
    if (argc == 3){
        mesh.LoadObjFile(argv[1]);
        cout << mesh.vList.size() << endl;
        mesh.SetConstraints(argv[2]);
    }
    else{
        cout << "Usage: ./main ${Obj_path} ${Anchors_path}" << endl;
    }
    SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());

    glutMainLoop();

    return 0;
}

