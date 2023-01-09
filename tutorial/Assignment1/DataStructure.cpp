#include <per_vertex_normals.h>
#include "DataStructure.h"
#include "Model.h"
#include <utility>


DataStructure::DataStructure(std::shared_ptr<cg3d::Mesh> mesh) : originalMesh(mesh) {
    OV = mesh->data[0].vertices;
    OF = mesh->data[0].faces;
}

std::shared_ptr<cg3d::Mesh> DataStructure::resetMesh(igl::opengl::glfw::Viewer* viewer) {
    V = OV;
    F = OF;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    C.resize(E.rows(), V.cols());
    Eigen::VectorXd costs(E.rows());
    q = {};
    EQ = Eigen::VectorXi::Zero(E.rows());
    {
        Eigen::VectorXd costs(E.rows());
        igl::parallel_for(E.rows(), [&](const int e)
            {
                double cost = e;
        Eigen::RowVectorXd p(1, 3);
        igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
        C.row(e) = p;
        costs(e) = cost;
            }, 10000);
        for (int e = 0; e < E.rows(); e++)
        {
            q.emplace(costs(e), e, 0);
        }
    }

    collapsed_faces_counter = 0;
    Eigen::MatrixXd vertexNormals;
    igl::per_vertex_normals(V, F, vertexNormals);
    std::vector<cg3d::MeshData> newMeshDataList;
    newMeshDataList.push_back({ V, F, vertexNormals, Eigen::MatrixXd::Zero(V.rows(), 2) });
    return std::make_shared<cg3d::Mesh>("new mesh", newMeshDataList);
}


std::shared_ptr<cg3d::Mesh> DataStructure::simplifyTenPercent(igl::opengl::glfw::Viewer* viewer) {

    resetCallbackParmas();

    int collapsedBefore = collapsed_faces_counter;

    if (!q.empty())
    {
        bool something_collapsed = false;

        for (int j = 0; j < std::ceil(q.size() * 0.01); j++)
        {
            auto next = q.top();
            auto cost = std::get<0>(next);
            auto v1Index = std::get<1>(next);
            auto v2Index = std::get<2>(next);
            auto v1 = V.row(v2Index);
            auto v2 = V.row(v2Index);

            if (!igl::collapse_edge(newCostCallback, V, F, E, EMAP, EF, EI, q, EQ, C))
                break;

            std::cout << "Collapsed edge in: (" << v1 << "), (" << v2 << ") with cost: " << cost << std::endl;
            std::cout << "New position is: (" << VTags[v1Index](0, 0) << " " << VTags[v1Index](1, 0) << " " << VTags[v1Index](2, 0) << "), (" <<
                VTags[v2Index](0, 0) << " " << VTags[v2Index](1, 0) << " " << VTags[v2Index](2, 0) << ")" << std::endl;

            something_collapsed = true;
            collapsed_faces_counter++;
        }

        if (something_collapsed)
        {
            Eigen::MatrixXd vertexNormals;
            igl::per_vertex_normals(V, F, vertexNormals);
            std::vector<cg3d::MeshData> newMeshDataList;
            newMeshDataList.push_back({ V, F, vertexNormals, Eigen::MatrixXd::Zero(V.rows(), 2) });

            return std::make_shared<cg3d::Mesh>("new mesh", newMeshDataList);
        }
    }
    return nullptr;
}

void DataStructure::resetCallbackParmas() {
    Eigen::MatrixXd vertexNormals;
    igl::per_vertex_normals(V, F, vertexNormals);


    for (int i = 0; i < V.rows(); i++) {
        Eigen::MatrixXd normal = vertexNormals.row(i);
        Eigen::MatrixXd transposedNormal = normal.transpose();
        Eigen::MatrixXd vertex = V.row(i);

        double dVal = -(transposedNormal * vertex)(0, 0);
        double dValSquare = dVal * dVal;
        Eigen::DiagonalMatrix<double, 3> diag(dValSquare, dValSquare, dValSquare);

        Eigen::MatrixXd q1q2 = vertex.transpose() * (normal * transposedNormal) * vertex + static_cast<Eigen::MatrixXd>(2 * (dVal * normal).transpose()) * vertex;      
        Eigen::MatrixXd q = static_cast<Eigen::MatrixXd>(q1q2) + static_cast<Eigen::MatrixXd>(diag);

        Eigen::Matrix4d newQ;
        for (int k = 0; k < 3; k++) {
            for (int l = 0; l < 3; l++) {
                newQ(k, l) = q(k, l);
            }
        }

        newQ(3, 0) = 0;
        newQ(3, 1) = 0;
        newQ(3, 2) = 0;

        newQ(0, 3) = 0;
        newQ(1, 3) = 0;
        newQ(2, 3) = 0;

        newQ(3, 3) = 1;

        Qs[i] = newQ;
    }

    for (int j = 0; j < E.rows(); j++) {
        int i1 = E.row(j)[0];
        int i2 = E.row(j)[1];

        Eigen::MatrixXd qTag = Qs[i1] + Qs[i2];
        QTags[j] = qTag;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(qTag);
        Eigen::Vector4d res;
        if (lu.isInvertible()) {
            res = lu.inverse() * Eigen::Vector4d(0, 0, 0, 1);
        }
        else {
            Eigen::MatrixXd sum = (0.5 * static_cast<Eigen::MatrixXd>(V.row(i1) + V.row(i2)));
            res.x() = sum(0, 0);
            res.y() = sum(0, 1);
            res.z() = sum(0, 2);
            res.w() = 1;
        }

        VTags[j] = res;
    }



    newCostCallback = [this](const int e,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXi& E,
        const Eigen::VectorXi& EMAP,
        const Eigen::MatrixXi& EF,
        const Eigen::MatrixXi& EI,
        double& cost,
        Eigen::RowVectorXd& p) {
            Eigen::MatrixXd vTag = VTags.at(e);
            p = Eigen::Vector3d(vTag(0, 0), vTag(1, 0), vTag(2, 0));
            cost = (vTag.transpose() * QTags.at(e) * vTag)(0, 0);
    };



}