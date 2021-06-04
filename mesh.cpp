#include "mesh.hpp"
#include "matrix.hpp"
#include <cstring>
#include <iostream>
#include <strstream>
#include <fstream>
#include <cmath>
#include <float.h>
#include <assert.h>
#include <queue>
#include <list>

#define PI 3.1415926535

using namespace std;


/////////////////////////////////////////
// helping inline functions
inline double Cot(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
    Vector3d v1 = p1 - p2;
    Vector3d v2 = p3 - p2;

    v1 /= v1.L2Norm();
    v2 /= v2.L2Norm();
    double tmp = v1.Dot(v2);
    return 1.0 / tan(acos(tmp));
}

inline double Area(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
    Vector3d v1 = p2 - p1;
    Vector3d v2 = p3 - p1;
    return v1.Cross(v2).L2Norm() / 2.0;
}


/////////////////////////////////////////
// implementation of OneRingHEdge class
OneRingHEdge::OneRingHEdge(const Vertex * v) {
    if (v == NULL) start = next = NULL;
    else start = next = v->HalfEdge();
}

HEdge * OneRingHEdge::NextHEdge() {
    HEdge *ret = next;
    if (next && next->Prev()->Twin() != start)
        next = next->Prev()->Twin();
    else
        next = NULL;
    return ret;
}

/////////////////////////////////////////
// implementation of Mesh class
//
// function AddFace
// it's only for loading obj model, you do not need to understand it
void Mesh::AddFace(int v1, int v2, int v3) {
    int i;
    HEdge *he[3], *bhe[3];
    Vertex *v[3];
    Face *f;

    // obtain objects
    for (i=0; i<3; i++) he[i] = new HEdge();
    for (i=0; i<3; i++) bhe[i] = new HEdge(true);
    v[0] = vList[v1];
    v[1] = vList[v2];
    v[2] = vList[v3];
    f = new Face();

    // connect prev-next pointers
    SetPrevNext(he[0], he[1]);
    SetPrevNext(he[1], he[2]);
    SetPrevNext(he[2], he[0]);
    SetPrevNext(bhe[0], bhe[1]);
    SetPrevNext(bhe[1], bhe[2]);
    SetPrevNext(bhe[2], bhe[0]);

    // connect twin pointers
    SetTwin(he[0], bhe[0]);
    SetTwin(he[1], bhe[2]);
    SetTwin(he[2], bhe[1]);

    // connect start pointers for bhe
    bhe[0]->SetStart(v[1]);
    bhe[1]->SetStart(v[0]);
    bhe[2]->SetStart(v[2]);
    for (i=0; i<3; i++) he[i]->SetStart(v[i]);

    // connect start pointers
    // connect face-hedge pointers
    for (i=0; i<3; i++) {
        v[i]->SetHalfEdge(he[i]);
        v[i]->adjHEdges.push_back(he[i]);
        SetFace(f, he[i]);
    }
    v[0]->adjHEdges.push_back(bhe[1]);
    v[1]->adjHEdges.push_back(bhe[0]);
    v[2]->adjHEdges.push_back(bhe[2]);

    // mearge boundary if in need
    for (i=0; i<3; i++) {
        Vertex *start = bhe[i]->Start();
        Vertex *end   = bhe[i]->End();
        for (size_t j=0; j<end->adjHEdges.size(); j++) {
            HEdge *curr = end->adjHEdges[j];
            if (curr->IsBoundary() && curr->End()==start) {
                SetPrevNext(bhe[i]->Prev(), curr->Next());
                SetPrevNext(curr->Prev(), bhe[i]->Next());
                SetTwin(bhe[i]->Twin(), curr->Twin());
                bhe[i]->SetStart(NULL);	// mark as unused
                curr->SetStart(NULL);	// mark as unused
                break;
            }
        }
    }

    // finally add hedges and faces to list
    for (i=0; i<3; i++) heList.push_back(he[i]);
    for (i=0; i<3; i++) bheList.push_back(bhe[i]);
    fList.push_back(f);
}

// function LoadObjFile
// it's only for loading obj model, you do not need to understand it
bool Mesh::LoadObjFile(const char *filename) {
    if (filename==NULL || strlen(filename)==0) return false;

    ifstream ifs(filename);
    if (ifs.fail()) return false;
    Clear();
    vector<double> vertex_x, vertex_y, vertex_z;
    vector<int> face_idx1, face_idx2, face_idx3;

    char buf[1024], type[1024];
    do {
        ifs.getline(buf, 1024);
        istrstream iss(buf);
        memset(type, '\0', sizeof(type));
        iss >> type;

        // vertex
        if (strcmp(type, "v") == 0) {
            double x, y, z;
            iss >> x >> y >> z;
            AddVertex(new Vertex(x,y,z));
            vertex_x.push_back(x);
            vertex_y.push_back(y);
            vertex_z.push_back(z);
        }
            // face
        else if (strcmp(type, "f") == 0) {
            int index[3];
            iss >> index[0] >> index[1] >> index[2];
            AddFace(index[0]-1, index[1]-1, index[2]-1);
            face_idx1.push_back(index[0]-1);
            face_idx2.push_back(index[1]-1);
            face_idx3.push_back(index[2]-1);
        }
    } while (!ifs.eof());
    ifs.close();

    size_t i;
    Vector3d box = this->MaxCoord() - this->MinCoord();
    for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());

    Vector3d tot;
    for (i=0; i<vList.size(); i++) tot += vList[i]->Position();
    Vector3d avg = tot / vList.size();
    for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

    HEdgeList list;
    for (i=0; i<bheList.size(); i++)
        if (bheList[i]->Start()) list.push_back(bheList[i]);
    bheList = list;

    for (i=0; i<vList.size(); i++)
    {
        vList[i]->adjHEdges.clear();
        vList[i]->SetIndex((int)i);
        vList[i]->SetFlag(0);
    }

    p.resize(3, vList.size());
    faces.resize(3, fList.size());
    for(int i = 0; i < vList.size(); i++){
        p(0,i) = vertex_x[i];
        p(1,i) = vertex_y[i];
        p(2,i) = vertex_z[i];
    }
    for(int i = 0; i < fList.size(); i++){
        faces(0,i) = face_idx1[i];
        faces(1,i) = face_idx2[i];
        faces(2,i) = face_idx3[i];
    }
    p_prime = p;

    return true;
}

void Mesh::SetAnchors(const char* filename) {
    assert(filename != nullptr && strlen(filename) > 0);
    ifstream ifs(filename);
    assert(ifs.fail() == false);

    char buf[1024];
    do{
        ifs.getline(buf, 1024);
        istrstream iss(buf);
        int idx;
        double x, y, z;
        iss >> idx >> x >> y >> z;
        auto ret = anchors.emplace(idx, Eigen::Matrix<double,3,1>(x,y,z));
        assert(ret.second);
    }while(!ifs.eof());
    ifs.close();
}

void Mesh::InitWeights(){
    vector<Eigen::Triplet<double>> weight_list;
    weight_list.reserve(fList.size()*3);

    for(int i = 0; i < vList.size(); i++){
        OneRingVertex ring(vList[i]);
        /* Get neighbor vertices  */
        vector<Vertex*> neighbors;
        Vertex *temp = nullptr;
        while((temp = ring.NextVertex())) {
            neighbors.push_back(temp);
        }

        /* Compute weight for each pair */
        Vector3d v = vList[i]->Position();
        Vector3d curr = neighbors[0]->Position();
        Vector3d next = neighbors[1]->Position();
        Vector3d prev = neighbors[neighbors.size()-1]->Position();
        double cot_alpha = Cot(v, prev, curr);
        double cot_beta = Cot(v, next, curr);
        double weight = (cot_alpha + cot_beta) / 2.0;
        weight_list.emplace_back(Eigen::Triplet<double>(i,neighbors[0]->Index(),weight));
        weight_list.emplace_back(Eigen::Triplet<double>(neighbors[0]->Index(),i,weight));

        for(int j = 1; j < neighbors.size() - 1; j++){
            curr = neighbors[j]->Position();
            next = neighbors[j+1]->Position();
            prev = neighbors[j-1]->Position();
            cot_alpha = Cot(v, prev, curr);
            cot_beta = Cot(v, next, curr);
            weight = (cot_alpha + cot_beta) / 2.0;
            weight_list.emplace_back(Eigen::Triplet<double>(i,neighbors[j]->Index(),weight));
            weight_list.emplace_back(Eigen::Triplet<double>(neighbors[j]->Index(),i,weight));
        }
        curr = neighbors[neighbors.size()-1]->Position();
        next = neighbors[0]->Position();
        prev = neighbors[neighbors.size()-2]->Position();
        cot_alpha = Cot(v, prev, curr);
        cot_beta = Cot(v, next, curr);
        weight = (cot_alpha + cot_beta) / 2.0;
        weight_list.emplace_back(Eigen::Triplet<double>(i,neighbors[neighbors.size()-1]->Index(),weight));
        weight_list.emplace_back(Eigen::Triplet<double>(neighbors[neighbors.size()-1]->Index(),i,weight));
    }

    weights.resize(vList.size(), vList.size());
    weights.setZero();
    weights.setFromTriplets(weight_list.begin(), weight_list.end());
}

void Mesh::InitRotations() {
    rotations.clear();
    rotations.resize(vList.size(), Eigen::Matrix3d::Identity());
}

void Mesh::InitHandleMapping() {
    handleMap.resize(vList.size());
    int index = 0;
    for(int i = 0; i < vList.size(); i++){
        if(anchors.find(i) != anchors.end()){
            handleMap[i] = -1;
        }
        else{
            handleMap[i] = index;
            index++;
        }
    }
}

void Mesh::BuildLinearSystem() {
    L.resize(handle_num, handle_num);
    L.reserve(Eigen::VectorXi::Constant(handle_num, 7));
    L.setZero();
    b.resize(3, handle_num);
    b.setZero();

    vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(7 * handle_num);

    for(int row_idx = 0; row_idx < weights.outerSize(); row_idx++){
        int handle_idx1 = handleMap[row_idx];
        if(handle_idx1 == -1){
            continue;
        }
        for(Eigen::SparseMatrix<double>::InnerIterator col_it(weights, row_idx); col_it; ++col_it){
            int col_idx = col_it.col();
            int handle_idx2 = handleMap[col_idx];
            double w = col_it.value();
            if(handle_idx2 == -1){
                b.col(handle_idx1) += w * anchors[col_idx];
            }
            else{
                triplets.emplace_back(Eigen::Triplet<double>(handle_idx1, handle_idx2, -w));
            }
            triplets.emplace_back(Eigen::Triplet<double>(handle_idx1, handle_idx1, w));
        }
    }
    L.setFromTriplets(triplets.begin(), triplets.end());
    solver.compute(L);
}

void Mesh::SolveLinearSystem() {
    Eigen::Matrix<double, Eigen::Dynamic, 1> x;
    for(int i = 0; i < 3; i++){
        x = solver.solve(b.row(i).transpose());

        int idx = 0;
        for(int j = 0; j < vList.size(); j++){
            if(handleMap[j] != -1){
                p_prime(i, j) = x(idx);
                idx++;
            }
        }
    }
}

void Mesh::EstimateRotations() {
    for(int row_idx = 0; row_idx < weights.outerSize(); row_idx++){
        auto pi = p.col(row_idx);
        auto ppi = p_prime.col(row_idx);
        /* Compute covariance matrix */
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for(Eigen::SparseMatrix<double>::InnerIterator col_it(weights, row_idx); col_it; ++col_it){
            int col_idx = col_it.col();
            double w = col_it.value();
            auto pj = p.col(col_idx);
            auto ppj = p_prime.col(col_idx);
            covariance += w * (pi - pj) * (ppi - ppj).transpose();
        }
        /* SVD Decomposition */
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Matrix3d Ut = svd.matrixU().transpose();

        /* Get rotation matrix */
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3,3);
        I(2,2) = (V * Ut).determinant();
        rotations[row_idx] = (V * I * Ut);
    }
}

void Mesh::EstimatePositions() {
    for(int row_idx = 0; row_idx < weights.outerSize(); row_idx++){
        int handle_idx = handleMap[row_idx];
        if(handle_idx == -1){
            continue;
        }
        for(Eigen::SparseMatrix<double>::InnerIterator col_it(weights, row_idx); col_it; ++col_it){
            int col_idx = col_it.col();
            double w = col_it.value();
            Eigen::Matrix3d R = rotations[row_idx] + rotations[col_idx];
            Eigen::Matrix<double, 3, 1> point = R * (p.col(row_idx) - p.col(col_idx)) * w * 0.5;
            b.col(handle_idx) += point;
        }
    }
}

void Mesh::UpdateVertices() {
    for(int i = 0; i < vList.size(); i++){
        auto pos = p_prime.col(i);
        vList[i]->SetPosition(Vector3d(pos[0], pos[1], pos[2]));
    }
}

void Mesh::Deform(int num_iterations){
    /* Build linear system */
    InitRotations();
    InitWeights();
    InitHandleMapping();
    BuildLinearSystem();

    /* Interleaved iterations */
    for(int i = 0; i < num_iterations; i++){
        EstimateRotations();
        EstimatePositions();
        SolveLinearSystem();
    }

    /* Update vertex positions */
    UpdateVertices();
}