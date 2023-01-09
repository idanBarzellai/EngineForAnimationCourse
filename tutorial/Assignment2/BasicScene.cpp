#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "opengl/ViewerData.h"
#include "igl/bounding_box.h"
#include "Eigen/Core"
#include "igl/per_vertex_normals.h"
#include <Eigen/Dense>
#include <igl/AABB.h>

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root"));
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
    this->core().is_animating = true;


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };
    auto material2{ std::make_shared<Material>("material2", program) };

    material->AddTexture(0, "textures/box0.bmp", 2);
    mat = material;
    bool isBunnies = true;
    scaleFactor = 1;

    if (isBunnies) { //true for bunny false for cube	
        // Cube	
        scaleFactor = 20;

        auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/bunny.off") };
        cube = Model::Create("cube", cubeMesh, material);
        cube->Translate({ 0, 0, 0 });
        cube->Scale(scaleFactor);
        cube->showWireframe = true;
        pickedModel = cube;
        for (int i = 0; i < 30; i++) {
            simplify();
        }
        // Cube2	
        auto cubeMesh2{ IglLoader::MeshFromFiles("cube_igl","data/bunny.off") };
        cube2 = Model::Create("cube2", cubeMesh2, material);
        cube2->Translate({ 5, 0, 0 });
        cube2->Scale(scaleFactor);
        cube2->showWireframe = true;
        pickedModel = cube2;
        for (int i = 0; i < 30; i++) {
            simplify();
        }
        pickedModel = NULL;

    }
    else { // For check
        // Cube
        auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
        cube = Model::Create("cube", cubeMesh, material);
        cube->Translate({ 0, 0, 0 });
        // Cube2	
        auto cubeMesh2{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
        cube2 = Model::Create("cube2", cubeMesh2, material);
        cube2->Translate({ 2, 0, 0 });
    }

    material2->AddTexture(0, "textures/grass.bmp", 2);
    auto cubeMeshBox{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    cubeBox = Model::Create("cubeBox", cubeMeshBox, material2);
    cubeBox->isPickable = false;

    auto cubeMesh2Box{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    cubeBox2 = Model::Create("cube2Box", cubeMesh2Box, material2);

    models.push_back(cube);
    models.push_back(cube2);
    kd_tree.init(cube->GetMesh()->data[0].vertices, cube->GetMesh()->data[0].faces);
    kd_tree2.init(cube2->GetMesh()->data[0].vertices, cube2->GetMesh()->data[0].faces);


    camera->Translate(20, Axis::Z);
    root->AddChild(cube);
    root->AddChild(cube2);
    cube->AddChild(cubeBox);
    cube2->AddChild(cubeBox2);

    drawAlignedBox(cubeBox, kd_tree.m_box);
    drawAlignedBox(cubeBox2, kd_tree2.m_box);

    cube->Translate({ 0.0000001, 0, 0 });
    cube2->Translate({ 0.000001, 0, 0 });

    isActive = false;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);

    if (isActive) {
        Animate();
    }
}

void BasicScene::simplify() {
    if (pickedModel == nullptr) return;
    if (dataStructures.find(pickedModel->name) == dataStructures.end()) createDataStructures(pickedModel);
    if (originalMeshes.find(pickedModel->name) == originalMeshes.end()) originalMeshes[pickedModel->name] = {};

    bool simplified = false;
    std::vector<std::shared_ptr<DataStructure>> modeldataStructures = dataStructures[pickedModel->name];

    std::vector<std::shared_ptr<cg3d::Mesh>> newMeshList;
    for (int i = 0; i < pickedModel->GetMeshList().size(); i++) {
        auto newMesh = modeldataStructures[i]->simplifyTenPercent(this);
        if (newMesh != nullptr) {
            newMeshList.push_back(newMesh);
            simplified = true;
        }
        else {
            newMeshList.push_back(pickedModel->GetMesh(i));
        }
    }
    originalMeshes[pickedModel->name].push(pickedModel->GetMeshList());
    pickedModel->SetMeshList(newMeshList);

}

void BasicScene::resetMesh() {
    if (pickedModel == nullptr || dataStructures.find(pickedModel->name) == dataStructures.end() || originalMeshes.find(pickedModel->name) == originalMeshes.end() || originalMeshes[pickedModel->name].empty()) return;
    auto mesh = originalMeshes[pickedModel->name].top();
    originalMeshes[pickedModel->name].pop();
    pickedModel->SetMeshList(mesh);
    createDataStructures(pickedModel);
}

void BasicScene::createDataStructures(std::shared_ptr<cg3d::Model> model) {
    std::vector<std::shared_ptr<DataStructure>> dataStructuresList;
    for (auto mesh : model->GetMeshList()) {
        auto dataStructure = std::make_shared<DataStructure>(mesh);
        dataStructure->resetMesh(this);
        dataStructuresList.push_back(dataStructure);
    }
    dataStructures[model->name] = dataStructuresList;

}

void BasicScene::KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {

        if (key == GLFW_KEY_SPACE) {

            drawAlignedBox(cubeBox, kd_tree.m_box);
            drawAlignedBox(cubeBox2, kd_tree2.m_box);
            isActive = !isActive;
        }
        else if (key == GLFW_KEY_LEFT_SHIFT) {
            resetMesh();
        }
        else if (key == GLFW_KEY_ENTER)
            draw();
    }
}

void BasicScene::Animate()
{
    Eigen::Vector3f step = Eigen::Vector3f(-0.001, 0, 0);

    if (isActive)
        if (check_collision()) {
            isActive = false;
        }
        else
            //for (auto& obj : models)	
            cube2->Translate(step);
}

IGL_INLINE bool BasicScene::check_collision() {
    return treeNodesCollide(kd_tree, kd_tree2);
}

IGL_INLINE bool BasicScene::treeNodesCollide(igl::AABB<Eigen::MatrixXd, 3>& firstObjNode, igl::AABB<Eigen::MatrixXd, 3>& secondObjNode) {

    if (boxes_collide(firstObjNode.m_box, secondObjNode.m_box)) {
        drawAlignedBox(cubeBox, firstObjNode.m_box);
        drawAlignedBox(cubeBox2, secondObjNode.m_box);
        if (firstObjNode.is_leaf() && secondObjNode.is_leaf()) {

            /* drawAlignedBox(cubeBox, firstObjNode.m_box);
             drawAlignedBox(cubeBox2, secondObjNode.m_box);*/
            return true;
        }
        else {
            if (firstObjNode.is_leaf()) {
                if (secondObjNode.m_left)
                    return treeNodesCollide(firstObjNode, *secondObjNode.m_left);
                if (secondObjNode.m_right)
                    return treeNodesCollide(firstObjNode, *secondObjNode.m_right);
            }
            else if (secondObjNode.is_leaf()) {
                if (firstObjNode.m_left)
                    return treeNodesCollide(*firstObjNode.m_left, secondObjNode);
                if (firstObjNode.m_right)
                    return treeNodesCollide(*firstObjNode.m_right, secondObjNode);
            }
            else
                return treeNodesCollide(*firstObjNode.m_left, *secondObjNode.m_left) ||
                treeNodesCollide(*firstObjNode.m_left, *secondObjNode.m_right) ||
                treeNodesCollide(*firstObjNode.m_right, *secondObjNode.m_left) ||
                treeNodesCollide(*firstObjNode.m_right, *secondObjNode.m_right);
        }
    }
    return false;
}

IGL_INLINE bool BasicScene::boxes_collide(Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox) {
    Eigen::Matrix3d firstRot, secRot;

    firstRot = models[0]->GetRotation().cast<double>();
    secRot = models[1]->GetRotation().cast<double>();

    double a0 = firstbox.sizes()[0] * scaleFactor / 2, a1 = firstbox.sizes()[1] * scaleFactor / 2, a2 = firstbox.sizes()[2] * scaleFactor / 2,
        b0 = secondbox.sizes()[0] * scaleFactor / 2, b1 = secondbox.sizes()[1] * scaleFactor / 2, b2 = secondbox.sizes()[2] * scaleFactor / 2,
        R0, R1, R;
    Eigen::Matrix3d A, B, C;
    Eigen::Vector3d D, C0, C1;
    Eigen::RowVector3d A0 = firstRot * Eigen::Vector3d(1, 0, 0),
        A1 = firstRot * Eigen::Vector3d(0, 1, 0),
        A2 = firstRot * Eigen::Vector3d(0, 0, 1),
        B0 = secRot * Eigen::Vector3d(1, 0, 0),
        B1 = secRot * Eigen::Vector3d(0, 1, 0),
        B2 = secRot * Eigen::Vector3d(0, 0, 1);
    A << Eigen::RowVector3d(A0[0], A1[0], A2[0]), Eigen::RowVector3d(A0[1], A1[1], A2[1]), Eigen::RowVector3d(A0[2], A1[2], A2[2]);
    B << Eigen::RowVector3d(B0[0], B1[0], B2[0]), Eigen::RowVector3d(B0[1], B1[1], B2[1]), Eigen::RowVector3d(B0[2], B1[2], B2[2]);
    C = A.transpose() * B;

    Eigen::Vector4f C0_4cord = (cube->Tout.matrix() * cube->Tin.matrix()).cast<float>() * Eigen::Vector4f(firstbox.center()[0], firstbox.center()[1], firstbox.center()[2], 1);
    Eigen::Vector4f C1_4cord = (cube2->Tout.matrix() * cube2->Tin.matrix()).cast<float>() * Eigen::Vector4f(secondbox.center()[0], secondbox.center()[1], secondbox.center()[2], 1);

    C0 = Eigen::Vector3d(C0_4cord[0], C0_4cord[1], C0_4cord[2]);
    C1 = Eigen::Vector3d(C1_4cord[0], C1_4cord[1], C1_4cord[2]);

    D = C1 - C0;



    //Table case 1
    R0 = a0;
    R1 = (b0 * abs(C(0, 0))) + (b1 * abs(C(0, 1))) + (b2 * abs(C(0, 2)));
    R = abs(A0.dot(D));

    if (R > R0 + R1) return false;

    //Table case 2
    R0 = a1;
    R1 = (b0 * abs(C(1, 0))) + (b1 * abs(C(1, 1))) + (b2 * abs(C(1, 2)));
    R = abs(A1.dot(D));

    if (R > R0 + R1) return false;

    //Table case 3
    R0 = a2;
    R1 = (b0 * abs(C(2, 0))) + (b1 * abs(C(2, 1))) + (b2 * abs(C(2, 2)));
    R = abs(A2.dot(D));

    if (R > R0 + R1) return false;

    //Table case 4
    R0 = (a0 * abs(C(0, 0))) + (a1 * abs(C(1, 0))) + (a2 * abs(C(2, 0)));
    R1 = b0;
    R = abs(B0.dot(D));

    if (R > R0 + R1) return false;

    //Table case 5
    R0 = (a0 * abs(C(0, 1))) + (a1 * abs(C(1, 1))) + (a2 * abs(C(2, 1)));
    R1 = b1;
    R = abs(B1.dot(D));

    if (R > R0 + R1) return false;

    //Table case 6
    R0 = (a0 * abs(C(0, 2))) + (a1 * abs(C(1, 2))) + (a2 * abs(C(2, 2)));
    R1 = b2;
    R = abs(B2.dot(D));

    if (R > R0 + R1) return false;

    //Table case 7
    R0 = (a1 * abs(C(2, 0))) + (a2 * abs(C(1, 0)));
    R1 = (b1 * abs(C(0, 2))) + (b2 * abs(C(0, 1)));
    R = abs((C(1, 0) * A2).dot(D) - (C(2, 0) * A1).dot(D));

    if (R > R0 + R1) return false;

    //Table case 8
    R0 = (a1 * abs(C(2, 1))) + (a2 * abs(C(1, 1)));
    R1 = (b0 * abs(C(0, 2))) + (b2 * abs(C(0, 0)));
    R = abs((C(1, 1) * A2).dot(D) - (C(2, 1) * A1).dot(D));

    if (R > R0 + R1) return false;

    //Table case 9
    R0 = (a1 * abs(C(2, 2))) + (a2 * abs(C(1, 2)));
    R1 = (b0 * abs(C(0, 1))) + (b1 * abs(C(0, 0)));
    R = abs((C(1, 2) * A2).dot(D) - (C(2, 2) * A1).dot(D));

    if (R > R0 + R1) return false;

    //Table case 10
    R0 = (a0 * abs(C(2, 0))) + (a2 * abs(C(0, 0)));
    R1 = (b1 * abs(C(1, 2))) + (b2 * abs(C(1, 1)));
    R = abs((C(2, 0) * A0).dot(D) - (C(0, 0) * A2).dot(D));

    if (R > R0 + R1) return false;

    //Table case 11
    R0 = (a0 * abs(C(2, 1))) + (a2 * abs(C(0, 1)));
    R1 = (b0 * abs(C(1, 2))) + (b2 * abs(C(1, 0)));
    R = abs((C(2, 1) * A0).dot(D) - (C(0, 1) * A2).dot(D));

    if (R > R0 + R1) return false;

    //Table case 12
    R0 = (a0 * abs(C(2, 2))) + (a2 * abs(C(0, 2)));
    R1 = (b0 * abs(C(1, 1))) + (b1 * abs(C(1, 0)));
    R = abs((C(2, 2) * A0).dot(D) - (C(0, 2) * A2).dot(D));

    if (R > R0 + R1) return false;

    //Table case 13
    R0 = (a0 * abs(C(1, 0))) + (a1 * abs(C(0, 0)));
    R1 = (b1 * abs(C(2, 2))) + (b2 * abs(C(2, 1)));
    R = abs((C(0, 0) * A1).dot(D) - (C(1, 0) * A0).dot(D));

    if (R > R0 + R1) return false;

    //Table case 14
    R0 = (a0 * abs(C(1, 1))) + (a1 * abs(C(0, 1)));
    R1 = (b0 * abs(C(2, 2))) + (b2 * abs(C(2, 0)));
    R = abs((C(0, 1) * A1).dot(D) - (C(1, 1) * A0).dot(D));

    if (R > R0 + R1) return false;

    //Table case 15
    R0 = (a0 * abs(C(1, 2))) + (a1 * abs(C(0, 2)));
    R1 = (b0 * abs(C(2, 1))) + (b1 * abs(C(2, 0)));
    R = abs((C(0, 2) * A1).dot(D) - (C(1, 2) * A0).dot(D));

    if (R > R0 + R1) return false;

    return true;
}






IGL_INLINE void BasicScene::drawAlignedBox(std::shared_ptr<cg3d::Model> cube, Eigen::AlignedBox<double, 3>& alignedBox) {



    Eigen::MatrixXd V(8, 3);
    Eigen::MatrixXi F(12, 3);
    V.row(1) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomLeftFloor);
    V.row(2) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor);
    V.row(5) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor);
    V.row(6) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopRightFloor);
    V.row(0) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil);
    V.row(3) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomRightCeil);
    V.row(4) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopLeftCeil);
    V.row(7) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopRightCeil);


    F = cube->GetMesh()->data[0].faces;

    Eigen::MatrixXd vertexNormals;

    igl::per_vertex_normals(V, F, vertexNormals);

    std::vector<cg3d::MeshData> newMeshDataList;
    std::vector<std::shared_ptr<cg3d::Mesh>> newMeshList;
    newMeshDataList.push_back({ V, F, vertexNormals, Eigen::MatrixXd::Zero(V.rows(), 2) });

    newMeshList.push_back(std::make_shared<cg3d::Mesh>("new mesh", newMeshDataList));
    cube->SetMeshList(newMeshList);

    cube->showWireframe = true;
    cube->showFaces = false;
    cube->isPickable = false;

}
