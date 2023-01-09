#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"


using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");

    auto material{ std::make_shared<Material>("material", program) }; // empty material
    auto material1{ std::make_shared<Material>("material", program1) }; // empty material

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/xcylinder.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj") };
    sphere1 = Model::Create("sphere", sphereMesh, material);
    cube = Model::Create("cube", cubeMesh, material);

    //Axis
    Eigen::MatrixXd vertices(6, 3);
    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
    Eigen::MatrixXi faces(3, 2);
    faces << 0, 1, 2, 3, 4, 5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
    axis.push_back(Model::Create("axis", coordsys, material1));
    axis[0]->mode = 1;
    axis[0]->Scale(4, Axis::XYZ);
    axis[0]->SetPickable(false);
    root->AddChild(axis[0]);

    scaleFactor = 1;

    cylCount = 4;
    delta = 0.05;
    isActive = false;
    max_dis = cylCount * 1.6 * scaleFactor;

    for (int i = 0; i < cylCount; i++)
    {
        cyls.push_back(Model::Create("cyl" + std::to_string(i), cylMesh, material));
        cyls[i]->Scale(scaleFactor, Axis::X);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::X);
        cyls[i]->SetCenter(Eigen::Vector3f(-0.8f * scaleFactor, 0, 0));

        std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i + 1]->mode = 1;
        axis[i + 1]->Scale(2, Axis::XYZ);
        axis[i + 1]->Translate(0.8f * scaleFactor, Axis::X);
        axis[i + 1]->SetPickable(false);

        if (i == 0) {
            cyls[i]->Translate(-1.6f * scaleFactor, Axis::X);

            root->AddChild(cyls[0]);
            cyls[0]->AddChild(axis[i + 1]);
        }
        else {
            cyls[i - 1]->AddChild(cyls[i]);
            cyls[i]->AddChild(axis[i + 1]);
        }
    }
    cyls[0]->Translate({ 0.8f * scaleFactor,0,0 });

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };



    sphere1->showWireframe = true;
    sphere1->Translate(Eigen::Vector3f(5, 0, 0));
    std::cout << sphere1->GetTranslation() << "      " << CalcTipPOs() << std::endl;

    dist = (sphere1->GetTranslation() - CalcTipPOs()).norm();

    camera->Translate(22, Axis::Z);
    root->AddChild(sphere1);

    cube->mode = 1;
    auto mesh = cube->GetMeshList();

    int num_collapsed;

    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    std::cout << "vertices: \n" << V << std::endl;
    std::cout << "faces: \n" << F << std::endl;

    std::cout << "edges: \n" << E.transpose() << std::endl;
    std::cout << "edges to faces: \n" << EF.transpose() << std::endl;
    std::cout << "faces to edges: \n " << EMAP.transpose() << std::endl;
    std::cout << "edges indices: \n" << EI.transpose() << std::endl;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    dist = (sphere1->GetTranslation() - CalcTipPOs()).norm();
    cube->Rotate(0.1f, Axis::XYZ);
    if (isActive) {
        if ((sphere1->GetTranslation() - (cyls[0]->GetTranslation() + cyls[0]->GetRotation() * Eigen::Vector3f(-0.8 * scaleFactor, 0, 0))).norm() > max_dis)
            std::cout << "cannot reach - bye" << std::endl;
        else if (delta <= dist)
            IKSolver();
    }
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;


        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
        pickedToutAtPress = pickedModel->GetTout();
    }
    else {
        camera->TranslateInSystem(system, { 0, 0, -float(yoffset) });
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                pickedModel->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
                pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
            }
        }
        else {
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff / 10.0f, float(yAtPress - y) / moveCoeff / 10.0f, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress = x;
        yAtPress = y;
    }
}

Eigen::Vector3f BasicScene::CalcTipPOs() {
    return cyls[cylCount - 1]->GetTranslation() + cyls[cylCount - 1]->GetRotation() * Eigen::Vector3f(0.8 * scaleFactor, 0, 0);
}

void BasicScene::Animate()
{
    isActive = !isActive;
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:  //'space' – starts and stops IK solver animation.
            //std::cout << "picked modle: " << (pickedModel?"true" : "false") << std::endl;
            if (cylCount * 1.6f >= dist)
                Animate();
            else {
                std::cout << "cannot reach" << std::endl;
                isActive = false;
            }
            break;
            //        }
        case GLFW_KEY_P:
            /*'p' – prints rotation matrices (phi, theta and psi, according to ZXZ Euler angles) of the
picked link. If no link is picked prints the rotation matrix of the whole scene. (Won’t
move when the first link rotates.*/
            pickedModel ? std::cout << "rotation of " << pickedModel->name << ":\n" << pickedModel->GetRotation() << "\n" << std::endl
                : std::cout << "rotation of scene:\n" << root->GetRotation() << "\n" << std::endl;
            break;
        case GLFW_KEY_T:
            std::cout << "arm tip position = \n" << CalcTipPOs() << std::endl;
            break;
        case GLFW_KEY_Y:  //'d' prints destination position
            std::cout << "destination: \n" << sphere1->GetTranslation()  << ")\n" << std::endl;
            break;
        case GLFW_KEY_N: { //’n’ pick the next link, or the first one in case the last link is picked
            if (pickedModel != NULL) {
                int i = pickedModel->name[3];
                i = (i + 1) % cylCount;
                pickedModel = cyls.at(i);
            }
            break;
        }
        case GLFW_KEY_O:
            std::cout << "--/-- Test: !! --\\--" << "\n" <<
                //cyls[0].get()->GetTranslation() + cyls[0].get()->GetRotation() * Eigen::Vector3f(-0.8 * scaleFactor,0, 0) <<
                sphere1->GetTranslation() <<
                "\n--\\ --Test : !!--/--" << std::endl;
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            pickedModel->RotateInSystem(system, 0.1f, Axis::X);
            break;
        case GLFW_KEY_DOWN:
            pickedModel->RotateInSystem(system, -0.1f, Axis::X);
            break;
        case GLFW_KEY_LEFT:
            pickedModel->RotateInSystem(system, 0.1f, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            pickedModel->RotateInSystem(system, -0.1f, Axis::Y);
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.1f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.1f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.1f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.1f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.1f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.1f });
            break;

        }
    }
}

IGL_INLINE Eigen::Matrix3f BasicScene::CalcParentsInverseRotation(int index) {
    Eigen::Matrix3f rot;
    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    for (int i = index - 1; i >= 0; i--)
        rot = cyls.at(i)->GetRotation().inverse() * rot;

    return rot;
}

IGL_INLINE void BasicScene::IKSolver() {
    Eigen::Vector3f D = sphere1->GetTranslation() ;

    for (int i = cylCount - 1; i >= 0; i--) {
        Eigen::Vector3f E = CalcTipPOs();
        Eigen::Vector3f R = cyls[i]->GetTranslation() + (cyls[i] ->GetRotation() * Eigen::Vector3f(-0.8 * scaleFactor, 0, 0));
        const Eigen::Vector3f RE = (E - R).normalized();
        const Eigen::Vector3f RD = (D - R).normalized();

        float cosAngle = RE.dot(RD);
        if (cosAngle > 1) {
            cosAngle = 1;
        }
        if (cosAngle < -1) {
            cosAngle = -1;
        }
        float distance = (D - E).norm();
        dist = distance;

        float angleBetween = acos(cosAngle);

        Eigen::Vector3f rotationAxis = (RE.cross(RD)).normalized();
        rotationAxis = CalcParentsInverseRotation(i) * rotationAxis;

        cyls.at(i)->Rotate(angleBetween, rotationAxis);
    }
    if (delta > dist)  std::cout << "Distance is: " << dist << "\n" << std::endl;


}
