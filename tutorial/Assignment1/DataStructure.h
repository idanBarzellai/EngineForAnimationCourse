#include <Eigen/Core>
#include <igl/min_heap.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/collapse_edge.h>

#include "glfw/Viewer.h"
#include "Mesh.h"
#include "Model.h"

#ifndef ENGINEREWORK_DATASTRUCTURE_H
#define ENGINEREWORK_DATASTRUCTURE_H
class ExceptionHandler : public std::exception {};


class DataStructure {
private:
    Eigen::MatrixXd V, OV;
    Eigen::MatrixXi F, OF;
    Eigen::VectorXi EQ;
    Eigen::MatrixXi E;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi EF;
    Eigen::MatrixXi EI;
    igl::min_heap< std::tuple<double, int, int> > q;
    Eigen::MatrixXd C;
    int collapsed_faces_counter;
    std::map<int, Eigen::MatrixXd> Qs;
    std::map<int, Eigen::MatrixXd> VTags;
    std::map<int, Eigen::MatrixXd> QTags;
    void resetCallbackParmas();

public:
    DataStructure(std::shared_ptr<cg3d::Mesh> mesh);
    std::shared_ptr<cg3d::Mesh> originalMesh;
    std::shared_ptr<cg3d::Mesh> simplifyTenPercent(igl::opengl::glfw::Viewer* viewer);
    std::shared_ptr<cg3d::Mesh> resetMesh(igl::opengl::glfw::Viewer* viewer);
    igl::decimate_cost_and_placement_callback newCostCallback;

};


#endif //ENGINEREWORK_DATASTRUCTURE_H