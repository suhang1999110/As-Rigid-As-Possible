#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <GL/glut.h>
#include "mesh.hpp"
#include "matrix.hpp"
#include "OpenGLProjector.hpp"
#include <queue>

// Enumeration
enum EnumDisplayMode { WIREFRAME, HIDDENLINE, FLATSHADED, SMOOTHSHADED, COLORSMOOTHSHADED };

enum Mode
{
    Viewing,
    Selection,
    Dragging
};
Mode currentMode = Viewing;
int weight_type = UNIFORM;

bool VIS_HANDLE = true;
int ITER = 5;

// variables
int displayMode = FLATSHADED;	// current display mode
int mainMenu, displayMenu, weightMenu;		// glut menu handlers
int winWidth, winHeight;		// window width and height
double winAspect;				// winWidth / winHeight;
int lastX, lastY;				// last mouse motion position
int currSelectedVertex = -1;    // current selected vertex
bool leftDown, middleDown, middleUp, shiftDown, leftUp;		// mouse down and shift down flags
double sphi = 90.0, stheta = 45.0, sdepth = 10;	// for simple trackball
double xpan = 0.0, ypan = 0.0;				// for simple trackball
double zNear = 1.0, zFar = 100.0;
double g_fov = 45.0;
Vector3d g_center;
double g_sdepth;
double drag_start_x = 0, drag_start_y = 0;
Mesh mesh;	// our mesh
const double scale_factor = 0.1;

// UI variables
vector<int> anchor_indices;
vector<int> handle_indices;
vector<int> grouped_anchor_indices;
vector<int> newly_added_anchor_indices;

// functions
void MoveAnchors(Vector3d);
void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax);
void InitGL();
void InitMenu();
void InitGeometry();
void MenuCallback(int value);
void ReshapeFunc(int width, int height);
void DisplayFunc();
void DrawWireframe();
void DrawHiddenLine();
void DrawFlatShaded();
void DrawSmoothShaded();
void DrawColorSmoothShaded();
void DrawSelectedVertices();

void KeyboardFunc(unsigned char ch, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void SelectVertexByPoint();
void DeleteSelectedVertex(int vertex);
Eigen::Vector3d unproject(int x, int y);

void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax) {
    double PI = 3.14159265358979323846;
    double radius = bmax.Distance(bmin);
    g_center = 0.5 * (bmin+bmax);
    zNear    = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
    zFar     = zNear + 2.0 * radius;
    g_sdepth = zNear + radius;
    zNear *= 0.1;
    zFar *= 10;
    sdepth = g_sdepth;
}

// init openGL environment
void InitGL() {
    GLfloat light0Position[] = { 0, 1, 0, 1.0 };

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("CS271 Mesh Viewer");
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glPolygonOffset(1.0, 1.0);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glLightfv (GL_LIGHT0, GL_POSITION, light0Position);
    glEnable(GL_LIGHT0);

    glutReshapeFunc(ReshapeFunc);
    glutDisplayFunc(DisplayFunc);
    glutKeyboardFunc(KeyboardFunc);
    glutMouseFunc(MouseFunc);
    glutMotionFunc(MotionFunc);
}

// init right-click menu
void InitMenu() {
    displayMenu = glutCreateMenu(MenuCallback);
    glutAddMenuEntry("Wireframe", WIREFRAME);
    glutAddMenuEntry("Hidden Line", HIDDENLINE);
    glutAddMenuEntry("Flat Shaded", FLATSHADED);
    glutAddMenuEntry("Smooth Shaded", SMOOTHSHADED);
    glutAddMenuEntry("Color Smooth Shaded", COLORSMOOTHSHADED);

    weightMenu = glutCreateMenu(MenuCallback);
    glutAddMenuEntry("Uniform", UNIFORM);
    glutAddMenuEntry("Cotangent", COTANGENT);

    mainMenu = glutCreateMenu(MenuCallback);
    glutAddSubMenu("Display", displayMenu);
    glutAddSubMenu("Weight", weightMenu);
    glutAddMenuEntry("Exit", 99);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

// GLUT menu callback function
void MenuCallback(int value) {
    switch (glutGetMenu()) {
        case 1:
            displayMode = value;
            glutPostRedisplay();
            break;
        case 2:
            weight_type = value;
            glutPostRedisplay();
            break;
        case 3:
            exit(0);
            break;
    }

//    switch (value) {
//        case 99: exit(0); break;
//        default:
//            displayMode = value;
//            glutPostRedisplay();
//            break;
//    }
}

// GLUT reshape callback function
void ReshapeFunc(int width, int height) {
    winWidth = width;
    winHeight = height;
    winAspect = (double)width/(double)height;
    glViewport(0, 0, width, height);
    glutPostRedisplay();
}

// GLUT display callback function
void DisplayFunc() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(g_fov, winAspect, zNear, zFar);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(xpan, ypan, -sdepth);
    glRotatef(-stheta, 1.0, 0.0, 0.0);
    glRotatef(sphi, 0.0, 1.0, 0.0);
    glTranslatef(-g_center[0], -g_center[1], -g_center[2]);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    switch (displayMode) {
        case WIREFRAME: DrawWireframe(); break;
        case HIDDENLINE: DrawHiddenLine(); break;
        case FLATSHADED: DrawFlatShaded(); break;
        case SMOOTHSHADED: DrawSmoothShaded(); break;
        case COLORSMOOTHSHADED: DrawColorSmoothShaded(); break;
    }

    DrawSelectedVertices();

    glutSwapBuffers();
}

// Wireframe render function
void DrawWireframe() {
    HEdgeList heList = mesh.Edges();
    HEdgeList bheList = mesh.BoundaryEdges();
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    size_t i;
    for (i=0; i<heList.size(); i++) {
        glVertex3dv(heList[i]->Start()->Position().ToArray());
        glVertex3dv(heList[i]->End()->Position().ToArray());
    }
    glColor3f(1, 0, 0);
    for (i=0; i<bheList.size(); i++) {
        glVertex3dv(bheList[i]->Start()->Position().ToArray());
        glVertex3dv(bheList[i]->End()->Position().ToArray());
    }
    glEnd();
}

// Hidden Line render function
void DrawHiddenLine() {
    FaceList fList = mesh.Faces();
    glShadeModel(GL_FLAT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glColor3f(0, 0, 0);
    glBegin(GL_TRIANGLES);
    for (size_t i=0; i<fList.size(); i++) {
        Face *f = fList[i];
        const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
        const Vector3d & pos2 = f->HalfEdge()->End()->Position();
        const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
        glVertex3dv(pos1.ToArray());
        glVertex3dv(pos2.ToArray());
        glVertex3dv(pos3.ToArray());
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);

    DrawWireframe();
}

// Flat Shaded render function
void DrawFlatShaded() {
    FaceList fList = mesh.Faces();
    glShadeModel(GL_FLAT);
    glEnable(GL_LIGHTING);
    glColor3f(0.4f, 0.4f, 1.0f);
    glBegin(GL_TRIANGLES) ;
    for (size_t i=0; i<fList.size(); i++) {
        Face *f = fList[i];
        const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
        const Vector3d & pos2 = f->HalfEdge()->End()->Position();
        const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
        Vector3d normal = (pos2-pos1).Cross(pos3-pos1);
        normal /= normal.L2Norm();
        glNormal3dv(normal.ToArray());
        glVertex3dv(pos1.ToArray());
        glVertex3dv(pos2.ToArray());
        glVertex3dv(pos3.ToArray());
    }
    glEnd();
    glDisable(GL_LIGHTING);
}

// Smooth Shaded render function
void DrawSmoothShaded() {
    FaceList fList = mesh.Faces();
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glColor3f(0.4f, 0.4f, 1.0f);
    glBegin(GL_TRIANGLES) ;
    for (size_t i=0; i<fList.size(); i++) {
        Face *f = fList[i];
        Vertex * v1 = f->HalfEdge()->Start();
        Vertex * v2 = f->HalfEdge()->End();
        Vertex * v3 = f->HalfEdge()->Next()->End();
        glNormal3dv(v1->Normal().ToArray());
        glVertex3dv(v1->Position().ToArray());
        glNormal3dv(v2->Normal().ToArray());
        glVertex3dv(v2->Position().ToArray());
        glNormal3dv(v3->Normal().ToArray());
        glVertex3dv(v3->Position().ToArray());
    }
    glEnd();
    glDisable(GL_LIGHTING);
}

// Color Smooth Shaded render function
void DrawColorSmoothShaded() {
    FaceList fList = mesh.Faces();
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glColor3f(0.4f, 0.4f, 1.0f);
    glBegin(GL_TRIANGLES) ;
    for (size_t i=0; i<fList.size(); i++) {
        Face *f = fList[i];
        Vertex * v1 = f->HalfEdge()->Start();
        Vertex * v2 = f->HalfEdge()->End();
        Vertex * v3 = f->HalfEdge()->Next()->End();
        glNormal3dv(v1->Normal().ToArray());
        glColor3dv(v1->Color().ToArray());
        glVertex3dv(v1->Position().ToArray());
        glNormal3dv(v2->Normal().ToArray());
        glColor3dv(v2->Color().ToArray());
        glVertex3dv(v2->Position().ToArray());
        glNormal3dv(v3->Normal().ToArray());
        glColor3dv(v3->Color().ToArray());
        glVertex3dv(v3->Position().ToArray());
    }
    glEnd();
    glDisable(GL_LIGHTING);
}

// draw the selected ROI vertices on the mesh
void DrawSelectedVertices() {
    VertexList vList = mesh.Vertices();
    glColor3f(1.0, 0.0, 0.0);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    size_t i;
    if (currentMode != Dragging) {
        for (i = 0; i < vList.size(); i++) {
            if (vList[i]->Flag() == 1) glColor3f(0.8, 0.0, 0.0); // Mouse Selected Point
            else if (vList[i]->Type() == ANCHOR || vList[i]->Type() == STATIONARY) glColor3f(0.0, 0.0, 1.0); // Anchor Point
            else if (vList[i]->Type() == HANDLE) { // Handle Point
                if (VIS_HANDLE == 1) glColor3f(1.0, 1.0, 1.0); 
                else continue;
            }
            glVertex3dv(vList[i]->Position().ToArray());
        }
    } else {
        for (i = 0; i < vList.size(); i++) {
            vector<int>::iterator iter = find(grouped_anchor_indices.begin(), grouped_anchor_indices.end(), i);
            if (iter != grouped_anchor_indices.end()) glColor3f(0.8, 0.0, 0.0); // Groupped Anchor Point
            else if (vList[i]->Type() == ANCHOR || vList[i]->Type() == STATIONARY) glColor3f(0.0, 0.0, 1.0); // Anchor Point
            else if (vList[i]->Type() == HANDLE) { // Handle Point
                if (VIS_HANDLE == 1) glColor3f(1.0, 1.0, 1.0); 
                else continue;
            }
            glVertex3dv(vList[i]->Position().ToArray());
        }
    }
    glEnd();
}

// Drag a group of anchors
void MoveAnchors(Vector3d shift) {
    for (int idx: grouped_anchor_indices) {
        Vector3d pos = mesh.Vertices()[idx]->Position();
        mesh.Vertices()[idx]->SetPosition( pos + shift );
    }
}

// GLUT keyboard callback function
void KeyboardFunc(unsigned char ch, int x, int y) {
    Vector3d volume = mesh.MaxCoord() - mesh.MinCoord();
    double mean_volumn = (volume[0] + volume[1] + volume[2]) / 3;
    switch (ch) {
        case '1':	// key '1'
            currentMode = Viewing;
            cout << "Viewing mode" << endl;
            break;
        case '2':	// key '2'
            currentMode = Selection;
            cout << "Selection mode" << endl;
            break;
        case '3':   // key '3'
            currentMode = Dragging;
            cout << "Dragging mode" << endl;
            break;
        case 'a':   // Pick one anchor point
            if (currentMode == Selection && currSelectedVertex != -1) {
                if (mesh.Vertices()[currSelectedVertex]->Type() != HANDLE) {
                    grouped_anchor_indices.clear();
                    grouped_anchor_indices.emplace_back(currSelectedVertex);
                    deque<int> queue;
                    queue.push_back(currSelectedVertex);
                    while ( queue.size() != 0 ) {
                        OneRingVertex ring(mesh.Vertices()[queue.front()]);
                        Vertex *curr_neighbor = NULL;
                        queue.pop_front();
                        while ( curr_neighbor = ring.NextVertex() ) {
                            if (curr_neighbor->Type() != HANDLE) {
                                int idx = curr_neighbor->Index();
                                vector<int>::iterator iter = find(grouped_anchor_indices.begin(), grouped_anchor_indices.end(), idx);
                                if (iter == grouped_anchor_indices.end()) {
                                    queue.push_back(idx);
                                    grouped_anchor_indices.emplace_back(idx);
                                }
                            }
                        }
                    }
                } else {
                    anchor_indices.emplace_back(currSelectedVertex);
                    handle_indices.erase(remove(handle_indices.begin(), handle_indices.end(), currSelectedVertex), handle_indices.end());
                    grouped_anchor_indices.clear();
                    grouped_anchor_indices.emplace_back(currSelectedVertex);
                    newly_added_anchor_indices.clear();
                    newly_added_anchor_indices.emplace_back(currSelectedVertex);
                    mesh.Vertices()[currSelectedVertex]->SetFlag(0);
                    mesh.Vertices()[currSelectedVertex]->SetType(ANCHOR);
                }
            }
            break;
        case 'A':   // Extend a group of neighboring anchor points
            if (currentMode == Selection && currSelectedVertex != -1) {
                vector<int> iterated_list = newly_added_anchor_indices;
                newly_added_anchor_indices.clear();
                
                for (int idx: iterated_list) {
                    OneRingVertex ring(mesh.Vertices()[idx]);
                    Vertex *curr_neighbor = NULL;
                    while ( curr_neighbor = ring.NextVertex() ) {
                        if (curr_neighbor->Type() == HANDLE) {
                            curr_neighbor->SetFlag(0);
                            curr_neighbor->SetType(ANCHOR);
                            anchor_indices.emplace_back(curr_neighbor->Index());
                            handle_indices.erase(remove(handle_indices.begin(), handle_indices.end(), curr_neighbor->Index()), handle_indices.end());
                            grouped_anchor_indices.emplace_back(curr_neighbor->Index());
                            newly_added_anchor_indices.emplace_back(curr_neighbor->Index());
                        }
                    }
                }
            }
            break;
        case 'h':
        case 'H':   // Change anchor to handle
            if (currentMode == Selection && currSelectedVertex != -1) {
                handle_indices.emplace_back(currSelectedVertex);
                anchor_indices.erase(remove(anchor_indices.begin(), anchor_indices.end(), currSelectedVertex), anchor_indices.end());
                grouped_anchor_indices.clear();
                newly_added_anchor_indices.clear();
                mesh.Vertices()[currSelectedVertex]->SetFlag(0);
                mesh.Vertices()[currSelectedVertex]->SetType(HANDLE);
            }
            break;
        case 27:
            exit(0);
            break;
        case 'I':
        case 'i':   // key 'x-axis UP'
            if (currentMode == Dragging) MoveAnchors(Vector3d(scale_factor * mean_volumn, 0, 0));
            break;
        case 'K':
        case 'k':   // key 'x-axis DOWN'
            if (currentMode == Dragging) MoveAnchors(Vector3d(-scale_factor * mean_volumn, 0, 0));
            break;
        case 'J':
        case 'j':   // key 'y-axis UP'
            if (currentMode == Dragging) MoveAnchors(Vector3d(0, scale_factor * mean_volumn, 0));
            break;
        case 'L':
        case 'l':   // key 'y-axis DOWN'
            if (currentMode == Dragging) MoveAnchors(Vector3d(0, -scale_factor * mean_volumn, 0));
            break;
        case 'U':
        case 'u':   // key 'z-axis UP'
            if (currentMode == Dragging) MoveAnchors(Vector3d(0, 0, scale_factor * mean_volumn));
            break;
        case 'O':
        case 'o':   // key 'z-axis DOWN'
            if (currentMode == Dragging) MoveAnchors(Vector3d(0, 0, -scale_factor * mean_volumn));
            break;
        case '0':
            if (VIS_HANDLE == 1) VIS_HANDLE = 0;
            else VIS_HANDLE = 1;
            break;
        case '4':
            cout << "Deforming the mesh" << endl;
            mesh.SetConstraints(anchor_indices);
            cout<<anchor_indices.size()<<" "<<handle_indices.size()<<endl;
            mesh.Deform(ITER, static_cast<WEIGHT_TYPE>(weight_type));
            break;
        // case 'r':
        //     cout << "The picked point's index is " << currSelectedVertex << ".\n";
        //     break;
    }
    glutPostRedisplay();
}

// GLUT mouse callback function
void MouseFunc(int button, int state, int x, int y) {

    lastX = x;
    lastY = y;
    leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
    leftUp = (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP);
    middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
    middleUp = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_UP);
    shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);

    if (currentMode == Selection && state == GLUT_UP)
    {
        if (middleUp)
        {
            if (currSelectedVertex != -1)
            {
                mesh.Vertices()[currSelectedVertex]->SetFlag(0);
                currSelectedVertex = -1;
            }
        }
        else
        {
            SelectVertexByPoint();
        }

        lastX = lastY = 0;
        glutPostRedisplay();
    }
    if(currentMode == Dragging) {
        if(leftDown){
            drag_start_x = x;
            drag_start_y = y;
        }
        if(leftUp){
            cout<<"start: "<<drag_start_x<<" "<<drag_start_y<<endl;
            cout<<"end: "<<lastX<<" "<<lastY<<endl;
            auto start = unproject(drag_start_x, drag_start_y);
            auto end = unproject(lastX, lastY);
            auto shiftVec = end - start;
            MoveAnchors(Vector3d(shiftVec(0), shiftVec(1), shiftVec(2)));
        }
    }

}

// GLUT mouse motion callback function
void MotionFunc(int x, int y) {
    if (leftDown && currentMode != Dragging)
        if(!shiftDown) { // rotate
            sphi += (double)(x - lastX) / 4.0;
            stheta += (double)(lastY - y) / 4.0;
        } else { // pan
            xpan += (double)(x - lastX)*sdepth/zNear/winWidth;
            ypan += (double)(lastY - y)*sdepth/zNear/winHeight;
        }
    // scale
    if (middleDown) sdepth += (double)(lastY - y) / 10.0;

    lastX = x;
    lastY = y;
    glutPostRedisplay();
}


// select a mesh point
void SelectVertexByPoint()
{
    // get the selection position
    int x = lastX, y = winHeight - lastY;
    Vector3d u(x,y,0);

    OpenGLProjector projector;

    VertexList vList = mesh.Vertices();

    double mindis = 1e6; int selectedIndex = -1;
    for (size_t i=0; i<vList.size(); i++)
    {
        Vector3d v = projector.Project(vList[i]->Position());

        vList[i]->SetFlag(0); // set back to unselected

        if (projector.GetDepthValue((int)v.X(), (int)v.Y()) - v.Z() <= -1e-4)
        {
            continue;
        }
        v.Z() = 0;

        double dist = (u - v).L2Norm();
        if (dist < mindis)
        {
            mindis = dist;
            selectedIndex = (int)i;
        }
    }

    if (selectedIndex != -1)
    {
        currSelectedVertex = selectedIndex;
        vList[selectedIndex]->SetFlag(1);
        // cout<<"index: "<<vList[selectedIndex]->Index()<<" ";
        // cout<<"point: "<<vList[selectedIndex]->Position()<<endl;
    }

}

void InitAnchorList(){
    for(auto v: mesh.vList){
        if(v->Type() == ANCHOR){
            anchor_indices.push_back(v->Index());
            grouped_anchor_indices.push_back(v->Index());
        }
    }
}

Eigen::Vector3d unproject(int x, int y){
    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);
    GLdouble worldX, worldY, worldZ;
    gluUnProject(x, viewport[3]-y, 0, modelview, projection, viewport, &worldX, &worldY, &worldZ);
    return Eigen::Vector3d(worldX, worldY, worldZ);
}

#endif