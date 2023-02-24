#include <Windows.h>
#undef CreateWindow
#undef NEAR
#undef FAR
#undef near
#undef far
#include <MMSystem.h>
#pragma comment(lib, "winmm.lib")

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
#include "Model.h"

#define _USE_MATH_DEFINES
#include <math.h>

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



#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include "ObjLoader.h"
//#include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "Visitor.h"
#include <dqs.h>
#include <random>



using namespace cg3d;
void BasicScene::BuildImGui()
{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;

    ImGui::Begin("Menu", pOpen, flags);
    ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);

        ImGui::Text("Level: %d", level);
        ImGui::Text("Score: %d", score);

    ImGui::End();
}
BasicScene::BasicScene(std::string name, Display* display) : SceneWithImGui(std::move(name), display)
{
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
}

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);
    camList[0] = Camera::Create("cameraSnakeHead", fov, float(width) / float(height), near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/basicShader");
    
    mat = std::make_shared<Material>("material", program); // empty material
    mat2 = std::make_shared<Material>("material2", program1); // empty material

    auto material1{ std::make_shared<Material>("material", program1) }; // empty material
    auto grass{ std::make_shared<Material>("grass", program1) };
    grass->AddTexture(0, "textures/grass.bmp", 2);
    
    mat->AddTexture(0, "textures/box0.bmp", 2);
    mat2->AddTexture(1, "textures/snake.jpg", 2);

    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/xcylinder.obj") };
    meshCube = IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj");
    auto snakeMesh{ IglLoader::MeshFromFiles("snake_mesh", "data/snake1.obj") };
    auto snakeMesh2{ IglLoader::MeshFromFiles("snake_mesh", "data/snake2.obj") };
    

    camera->Translate(levelProperties, Axis::Z);
    

    targets.clear();
    kd_trees.clear();
    models.clear();

    scaleFactor = 1;

    cylCount =16;
    delta = 0.05;
    isActive = false;
    max_dis = cylCount * 1.6 * scaleFactor;

    targetMovementPos.push_back({ 1,0,0 });
    targetMovementPos.push_back({-1,0,0 });
    targetMovementPos.push_back({ 0,1,0 });
    targetMovementPos.push_back({ 0,-1,0 });
    
    std::vector<int> randomPlacesX = { -5, 6, -10, 11, 2, -8, -3, -10, 7 };
    std::vector<int> randomPlacesY = { -3, -5, 12, 3, 12, -10, 11,8, -7 };



    /*models.push_back(Model::Create("snake", snakeMesh, mat2));
    root->AddChild(models[0]);
    models[0]->Scale(3);
    igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(models[0]->GetMesh()->data[0].vertices, models[0]->GetMesh()->data[0].faces);
    kd_trees.push_back(kd_tree);*/

    models.push_back(Model::Create("floor", meshCube, grass));
    root->AddChild(models[0]);
    models[0]->Scale(levelProperties - 10);
    models[0]->Translate(Eigen::Vector3f(0, 0, -levelProperties/2));
    models[0]->isPickable = false;

   /* models.push_back(Model::Create("boundeires", meshCube, mat));
    root->AddChild(models[1]);
    models[1]->Scale(20);
    models[1]->Translate(Eigen::Vector3f(0, 0, 0));
    models[1]->isPickable = false;
    models[1]->showWireframe = true;
    models[1]->showFaces = false;
    */


    for (int i = 0; i < cylCount; i++)
    {
        cyls.push_back(Model::Create("cyls" + i, cylMesh, mat));
        i == 0 ? root->AddChild(cyls[i]) : cyls[i - 1]->AddChild(cyls[i]);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::X);
        cyls[i]->SetCenter(Eigen::Vector3f(-0.8f * scaleFactor, 0, 0));
        cyls[i]->isPickable = false;

        if (i == 0) {
            /*models.push_back(Model::Create("cylhead", meshCube, mat2));
            models.front()->isHidden = true;
            cyls[i - 1]->AddChild(cyls[i]);*/
            igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(cyls[i]->GetMesh()->data[0].vertices, cyls[i]->GetMesh()->data[0].faces);
            //kd_trees.push_back(kd_tree);
            kd_trreModelsMap.emplace(cyls[i], kd_tree);
        }
        
    }
    for (int i = 4; i < cylCount; i++)
    {
        cyls[i]->isHidden = true;
    }
    for (int i = 0; i < totalObjAmount; i++)
    {
        targets.push_back(Model::Create("newTarget" + i, meshCube, mat));
        root->AddChild(targets[i]);
        targets[i]->Translate(Eigen::Vector3f(randomPlacesX[i % 9], randomPlacesY[i % 9], 0));
        targets[i]->isHidden = true;
        targets[i]->isPickable = false;
        targetSides.push_back(false);
        


    }
    system("CLS");
    std::string answer;
    std::cout << "Do you want to start the Game? (Y/N): ";
    std::cin >> answer; // get user input from the keyboard
    if (!answer.compare("Y") || !answer.compare("y")) {
        std::cout << "Lets go!: " << std::endl;
        //foodCunter = 0;
    }
    else {
        std::cout << "OK bye... \nQuitting" << std::endl;
        exit(0);
    }

    cyls[0]->AddChild(camList[0]);

    camList[0]->Translate({0.5,0.5,0 });


}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{

    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);

    timer += 0.0001;
    speed = baseSpeed + (score * 0.001f);
    cyls[0]->TranslateInSystem(cyls[0]->GetRotation().transpose(), { -speed, 0, 0 });
    Animate();
    

    if (timer - lastTimeGenerated > generatingTime) {
        igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(targets[currAmountOfObjs]->GetMesh()->data[0].vertices, targets[currAmountOfObjs]->GetMesh()->data[0].faces);
       //kd_trees.push_back(kd_tree);
        kd_trreModelsMap.emplace(targets[currAmountOfObjs], kd_tree);
        targets[currAmountOfObjs]->isHidden = false;
        currAmountOfObjs++;
        lastTimeGenerated = timer;
    }

    if (timer - changeSIdesTime > 0.5) {
        changeSidesNow = !changeSidesNow;
        changeSIdesTime = timer;
    }
    MoveTarget(changeSidesNow);
    
    if (timer  - lastGameOverTime > gameOverTime) {
        
        if(soundOn)
            PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/end_notice.wav"), NULL, SND_ASYNC);
        system("CLS");

        std::string answer;
        std::cout << "Do you want to continue the Game? (Y/N): ";
        std::cin >> answer; // get user input from the keyboard
        if (!answer.compare("Y") || !answer.compare("y")) {
            std::cout << "Lets go!: " << std::endl;
            //foodCunter = 0;
        }
        else {
            std::cout << "OK bye... \nQuitting" << std::endl;
            exit(0);
        }
        lastGameOverTime = timer;
    }


    

    if (score == 2) {
        score = 0;

        SetLevel(++level);
    }
    
    // Getting Point
    for (int i = 0; i < targets.size(); i++)
    {
        Eigen::Vector3f dist = targets[i]->GetTranslation() - cyls[0]->GetTranslation();
        if (sqrt((dist[0] * dist[0]) + (dist[1] * dist[1]) + (dist[2] * dist[2])) < 2) {
            if (check_collision()) {
                system("CLS");
                if (soundOn)
                    PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/point.wav"), NULL, SND_ASYNC);
                std::cout << "+ 1 !!!!" << std::endl;
            }
        }
    }

    // Losing game
    for (int i = 0; i < 4 + score; i++)
    {
        float boundries = (levelProperties - 10) / 2;
        bool outOfBounderied = cyls[i]->GetTranslation()[0] >= boundries || cyls[i]->GetTranslation()[0] <= -boundries || cyls[i]->GetTranslation()[1] >= boundries || cyls[i]->GetTranslation()[1] <= -boundries;

        if (outOfBounderied || score < 0) {
            if (soundOn)
                PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/LostGame.wav"), NULL, SND_ASYNC);
            system("CLS");

            std::string answer;
            std::cout << "You LOST! Thanks for playing!\nPress Enter to exit... ";
            std::cin >> answer; // get user input from the keyboard
            std::cout << "OK bye... \nQuitting" << std::endl;
            exit(0);
        }
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


void BasicScene::MoveTarget(bool changesides) {
    Eigen::Vector3f vecPos;
    Eigen::Vector3f vecDest;
    Eigen::Vector3f vecDest2;

    for (int i = 0; i < currAmountOfObjs; i++)
    {
        vecPos = targets[i]->GetTranslation();
        vecDest = targetMovementPos[(i * 2) % 4]; // modulo in number of pos
        vecDest2 = targetMovementPos[((i * 2) + 1) % 4];

        if(changesides)
            targets[i]->TranslateInSystem(root->GetRotation().transpose(), vecDest2 * speed / 2);
        else
            targets[i]->TranslateInSystem(root->GetRotation().transpose(), vecDest * speed / 2);
    }
    
}
void BasicScene::Animate()
{
    Eigen::Vector3f vec1 = Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f vec2;

    for (int i = cylCount - 1; i > 0; i--)
    {
        vec2 = cyls[i]->Tout.rotation() * Eigen::Vector3f(1, 0, 0);
        Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(vec2, vec1);
        quat = quat.slerp(0.99, Eigen::Quaternionf::Identity());
        cyls[i]->Rotate(Eigen::Matrix3f(quat));
    }
}

void BasicScene::SetLevel(int level) {
    levelProperties = levelProperties - 5;
    camera->Translate(-5, Axis::Z);
    models[0]->Scale(0.9);
    resetSnake();
   
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            viewport->camera = isSnakeHeadCamera ? camera : camList[0];
            isSnakeHeadCamera = !isSnakeHeadCamera;
            
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            cyls[0]->RotateInSystem(system, speed / radius, Axis::Z);
            cyls[1]->RotateInSystem(system, -speed / radius, Axis::Z);
            for (int i = 2; i < (4+score); i++)
                cyls[i]->RotateInSystem(system, -speed / radius, Axis::Z);
            break;
        case GLFW_KEY_DOWN:

            cyls[0]->RotateInSystem(system, -speed / radius, Axis::Z);
            cyls[1]->RotateInSystem(system, speed / radius, Axis::Z);
            for (int i = 2; i < (4 + score); i++)
                cyls[i]->RotateInSystem(system, speed / radius, Axis::Z);
            break;
        case GLFW_KEY_LEFT:
            cyls[0]->RotateInSystem(system, speed / radius, Axis::Y);
            cyls[1]->RotateInSystem(system, -speed / radius, Axis::Y);
            for (int i = 2; i < cylCount; i++)
                cyls[i]->RotateInSystem(system, -speed / radius, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            cyls[0]->RotateInSystem(system, -speed / radius, Axis::Y);
            cyls[1]->RotateInSystem(system, speed / radius, Axis::Y);
            for (int i = 2; i < cylCount; i++)
                cyls[i]->RotateInSystem(system, speed / radius, Axis::Y);

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

//IGL_INLINE Eigen::Matrix3f BasicScene::CalcParentsInverseRotation(int index) {
//    Eigen::Matrix3f rot;
//    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//
//    for (int i = index - 1; i >= 0; i--)
//        rot = cyls.at(i)->GetRotation().inverse() * rot;
//
//    return rot;
//}
//
//IGL_INLINE void BasicScene::FABRIK(Eigen::Vector3f D) {
//    //Eigen::Vector3f D = snakeFood.front()->GetTranslation();
//
//    for (int i = cylCount - 1; i >= 0; i--) {
//        Eigen::Vector3f E = Eigen::Vector3f().Zero();// TODO CalcTipPOs();
//        Eigen::Vector3f R = cyls[i]->GetTranslation() + (cyls[i]->GetRotation() * Eigen::Vector3f(-0.8 * scaleFactor, 0, 0));
//        const Eigen::Vector3f RE = (E - R).normalized();
//        const Eigen::Vector3f RD = (D - R).normalized();
//
//        float cosAngle = RE.dot(RD);
//        if (cosAngle > 1) {
//            cosAngle = 1;
//        }
//        if (cosAngle < -1) {
//            cosAngle = -1;
//        }
//        float distance = (D - E).norm();
//        dist = distance;
//
//        float angleBetween = acos(cosAngle);
//
//        Eigen::Vector3f rotationAxis = (RE.cross(RD)).normalized();
//        rotationAxis = CalcParentsInverseRotation(i) * rotationAxis;
//
//        cyls.at(i)->Rotate(angleBetween, rotationAxis);
//    }
//    if (delta > dist)  std::cout << "Distance is: " << dist << "\n" << std::endl;
//
//
//}
//
//void BasicScene::simplify() {
//    if(pickedModel == nullptr) return;
//    else if(dataStructures.find(pickedModel->name) == dataStructures.end()) {
//        createDataStructures(pickedModel);
//    }
//    std::vector<std::shared_ptr<DataStructure>> modeldataStructures = dataStructures[pickedModel->name];
//
//    std::vector<std::shared_ptr<cg3d::Mesh>> newMeshList;
//    for(int i=0; i<pickedModel->GetMeshList().size(); i++) {
//        auto newMesh = modeldataStructures[i]->simplifyTenPercent(this);
//        if(newMesh != nullptr) {
//            newMeshList.push_back(newMesh);
//        } else {
//            newMeshList.push_back(pickedModel->GetMesh(i));
//        }
//    }
//    pickedModel->SetMeshList(newMeshList);
//
//}
//
//void BasicScene::resetMesh() {
//    if(pickedModel == nullptr || dataStructures.find(pickedModel->name) == dataStructures.end()) return;
//    std::vector<std::shared_ptr<cg3d::Mesh>> newMeshList;
//    for(auto DataStructure : dataStructures[pickedModel->name]) {
//        newMeshList.push_back(DataStructure->resetMesh(this));
//    }
//    pickedModel->SetMeshList(newMeshList);
//}
//
//void BasicScene::createDataStructures(std::shared_ptr<cg3d::Model> model) {
//    std::vector<std::shared_ptr<DataStructure>> dataStructuresList;
//    for(auto mesh : model->GetMeshList()) {
//        auto dataStructure = std::make_shared<DataStructure>(mesh);
//        dataStructure->resetMesh(this);
//        dataStructuresList.push_back(dataStructure);
//    }
//    dataStructures[model->name] = dataStructuresList;
//
//}
//
IGL_INLINE bool BasicScene::check_collision() {
    if (kd_trreModelsMap.size() < 2)
        return false;
    for (int i = 0; i < kd_trreModelsMap.size() -1; i++) {
        std::shared_ptr<cg3d::Model> currModel = targets[i];
        igl::AABB<Eigen::MatrixXd, 3> currKd_tree = kd_trreModelsMap.at(targets[i]); //kd_trees[i + 1];
        if (treeNodesCollide(cyls[0], currModel, kd_trreModelsMap.at(cyls[0]), currKd_tree)) {
            score++;
            currModel->isHidden = true;
            kd_trreModelsMap.at(targets[i]).deinit();
            //targets.erase(targets.begin() + i);
            /*kd_trees.erase(kd_trees.begin() + i + 1); */

            /*if (soundOn)
                PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/end_notice.wav"), NULL, SND_ASYNC);*/
            cyls[3 + score]->isHidden = false;
            return true;
        }
    }
    return false;
}

void BasicScene::resetSnake() {
    for (int i = 4; i < cylCount; i++)
    {
        cyls[i]->isHidden = true;
    }
}
//
IGL_INLINE bool BasicScene::treeNodesCollide(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2, igl::AABB<Eigen::MatrixXd, 3>& firstObjNode, igl::AABB<Eigen::MatrixXd, 3>& secondObjNode) {

    if (boxes_collide(model1, model2, firstObjNode.m_box, secondObjNode.m_box)) {
        if (firstObjNode.is_leaf() && secondObjNode.is_leaf()) {
            return true;
        }
    else {
        if (firstObjNode.is_leaf()) {
            if (secondObjNode.m_left)
                return treeNodesCollide(model1, model2, firstObjNode, *secondObjNode.m_left);
            if (secondObjNode.m_right)
                return treeNodesCollide(model1, model2, firstObjNode, *secondObjNode.m_right);
        }
        else if (secondObjNode.is_leaf()) {
            if (firstObjNode.m_left)
                return treeNodesCollide(model1, model2, *firstObjNode.m_left, secondObjNode);
            if (firstObjNode.m_right)
                return treeNodesCollide(model1, model2, *firstObjNode.m_right, secondObjNode);
        }
        else
            return treeNodesCollide(model1, model2, *firstObjNode.m_left, *secondObjNode.m_left) ||
            treeNodesCollide(model1, model2, *firstObjNode.m_left, *secondObjNode.m_right) ||
            treeNodesCollide(model1, model2, *firstObjNode.m_right, *secondObjNode.m_left) ||
            treeNodesCollide(model1, model2, *firstObjNode.m_right, *secondObjNode.m_right);
        }
    }
    return false;
}

IGL_INLINE bool BasicScene::boxes_collide(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2, Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox) {
    Eigen::Matrix3d firstRot, secRot;

    firstRot = model1->GetRotation().cast<double>();
    secRot = model2->GetRotation().cast<double>();

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

    Eigen::Vector4f C0_4cord = model1->GetTransform().cast<float>() * Eigen::Vector4f(firstbox.center()[0], firstbox.center()[1], firstbox.center()[2], 1);
    Eigen::Vector4f C1_4cord = model2->GetTransform().cast<float>() * Eigen::Vector4f(secondbox.center()[0], secondbox.center()[1], secondbox.center()[2], 1);

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


IGL_INLINE void BasicScene::initiate_speed(int objAmount, std::shared_ptr<cg3d::Model> model1)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 50);

    double x = (distr(gen) - 25.0) / 50.0;
    double y = (distr(gen) - 25.0) / 50.0;
    double z = 0;

    double prob = distr(gen);
    prob < 10 ? z = 0.5 : z = 0;

    if (model1->type == Model::BEZIER) {
        srand((unsigned)time(0));
        Eigen::Vector3d spawner_positions[4];
        spawner_positions[0] = Eigen::Vector3d(8, 0, 8);
        spawner_positions[1] = Eigen::Vector3d(-8, 0, 8);
        spawner_positions[2] = Eigen::Vector3d(-8, 0, -8);
        spawner_positions[3] = Eigen::Vector3d(8, 0, -8);

        model1->speed = Eigen::Vector3d::Zero();
        int iSpawner = (rand() % 4);
        double spawnerX = spawner_positions[iSpawner].x();
        double spawnerZ = spawner_positions[iSpawner].z();

        int angle = (rand() % 270) - 90;

        Eigen::Matrix <double, 4, 3> spline_points = Eigen::Matrix <double, 4, 3>::Zero();

        Eigen::Vector4d p0, p1, p2, p3;
        p0 = p1 = p2 = p3 = Eigen::Vector4d::UnitW();

        for (int i = 0; i < 3; ++i) {
            p1[i] = rand() % 15 - 2;
            p2[i] = rand() % 15 - 2;
            p3[i] = rand() % 15 - 2;
        }

        double angel_rad = angle * M_PI / 180;

        Eigen::Matrix <double, 3, 4> trans;		//	x rotation and translation to spawner
        trans << cosf(angel_rad), 0, -sinf(angel_rad), spawnerX,
            0, 1, 0, 0,
            sinf(angel_rad), 0, cosf(angel_rad), spawnerZ;

        spline_points.row(0) = trans * p0;
        spline_points.row(1) = trans * p1;
        spline_points.row(2) = trans * p2;
        spline_points.row(3) = trans * p3;

        model1->bezier_points = spline_points;
        Eigen::Matrix4d	M;					// Blending functions matrix
        M << -1, 3, -3, 1,
            3, -6, 3, 0,
            -3, 3, 0, 0,
            1, 0, 0, 0;

        model1->MG = M * model1->bezier_points;
        model1->T << 0, 0, 0, 1;

        double t = 0;
        model1->final_dir = (model1->bezier_points.row(3) - model1->bezier_points.row(2)).normalized();
    }
    if (model1->type == Model::BOUNCY) {
        model1->speed = Eigen::Vector3d(x / 4.0, y / 20.0, -z);

        x > 0 ? model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(-0.06, 0, 0)) :
            model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(0.06, 0, 0));
    }
    else {

        if (objAmount < 4) {
            //model1->speed = Eigen::Vector3d::Zero();
            //set_colors(Eigen::RowVector3d(rand() % 2, rand() % 2, rand() % 2));
            objAmount == 0 ? model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(-3 * x / 8.0, -3 * z != 0 ? 0.25 : y / 5.0, 0)) :
                objAmount == 1 ? model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(-0.003, 0.003, 0)) :
                objAmount == 2 ? model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(0.03, -0.03, 0)) :
                model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(0.03, 0.03, 0));
        }
        else {
            model1->speed = Eigen::Vector3d(x / 8.0, z != 0 ? 0.25 : y / 5.0, -z);

            std::random_device pos_rd;
            std::mt19937 pos_gen(pos_rd());
            std::uniform_int_distribution<> pos_distr(0, 50);

            double pos_x = (pos_distr(pos_gen) - 25.0) / 5.0;
            double pos_y = (pos_distr(pos_gen) - 25.0) / 5.0;

            z != 0 ? model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(pos_x * x / 8.0, -4 * z != 0 ? 0.25 : y / 5.0, 0)) :
                model1->TranslateInSystem(model1->GetRotation(), Eigen::Vector3f(pos_x * x / 8.0, pos_y * z != 0 ? 0.25 : y / 5.0, 0));
        }

    }
}



//double z = snake_tail_start, snake_link_len = link_length / cylCount;
//for (int i = 0; i < cylCount; i++)
//{
//    cyls.push_back(Model::Create("snakeLink" + i, cylMesh, mat));
//    i == 0 ? root->AddChild(cyls[i]) : cyls[i - 1]->AddChild(cyls[i]);
//    cyls[0]->SetCenter(Eigen::Vector3f(1, 0, 0));
//    skeleton.push_back(z * Eigen::Vector3d::UnitZ());
//    z += snake_link_len;
//}

//// Weights calc
//Eigen::MatrixXd V = models[0]->GetMesh()->data.front().vertices;

//int vertexNum = V.rows();
//W.resize(vertexNum, cylCount + 1);

//double z1, w1, w2, lBound, uBound;
//for (int i = 0; i < vertexNum; ++i) {
//    z1 = 10 * V.row(i).z();
//    lBound = floor(z1);
//    uBound = ceil(z1);

//    w1 = abs(z1 - uBound);
//    w2 = 1 - w1;

//    Eigen::VectorXd Wi;
//    Wi.resize(cylCount + 1);

//    for (int j = 0; j < Wi.size(); ++j)
//    {
//        j == lBound + 8 ? Wi[lBound + 8] = w1 :
//            j == uBound + 8 ? Wi[uBound + 8] = w2 :
//            Wi[j] = 0;
//    }

//    W.row(i) = Wi;
//}

//models[0]->Rotate(M_PI / 2, Eigen::Vector3f::UnitY());
///*for (int i = 0; i < cylCount; ++i)
//{
//    split_snake.emplace_back();
//    Eigen::Vector3f rev = Eigen::Vector3f( (float)skeleton[i][2], (float)skeleton[i][1], (float)skeleton[i][0] );
//    split_snake[i].Translate(rev);
//}*/

////for (int i = 0; i < cylCount; i++)
////{
////    if (i == cylCount) {
////        cyls.push_back(Model::Create("cyl" + std::to_string(i), sphereMesh, mat));
////    }
////    else {
////        cyls.push_back(Model::Create("cyl" + std::to_string(i), cylMesh, mat));
////    }
////    //cyls[i]->Scale(scaleFactor, Axis::X);
////    cyls[i]->Translate(1.6f * scaleFactor, Axis::X);
////    cyls[i]->SetCenter(Eigen::Vector3f(-0.8f * scaleFactor, 0, 0));
////    cyls[i]->isPickable = false;
////    /*std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
////    axis.push_back(Model::Create("axis", coordsys, material1));
////    axis[i + 1]->mode = 1;
////    axis[i + 1]->Scale(2, Axis::XYZ);
////    axis[i + 1]->Translate(0.8f * scaleFactor, Axis::X);
////    axis[i + 1]->SetPickable(false);*/

////    if (i == 0) {
////        cyls[i]->Translate(-1.6f * scaleFactor, Axis::X);
////        cyls[i]->RotateByDegree(90, Axis::X);

////        root->AddChild(cyls[0]);
////        //cyls[0]->AddChild(axis[i + 1]);
////    }
////    else {
////        cyls[i - 1]->AddChild(cyls[i]);
////        //cyls[i]->AddChild(axis[i + 1]);
////    }

////    igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(cyls[i]->GetMesh()->data.front().vertices, cyls[i]->GetMesh()->data.front().faces);
////    kd_trees.push_back(kd_tree);

////    models.push_back(cyls[i]);
////    
////}
////cyls[0]->Translate({0,0,0 });
//////cyls[cylCount]->Translate({ 0,0,0 });





////cyls[cylCount -1]->AddChild(camList[0]);


//// Snakes head
///*cyls.push_back(Model::Create("snake_head", snakeMesh, mat));
//cyls[cylCount]->Scale(scaleFactor, Axis::X);
//cyls[cylCount]->Translate(1.6f * scaleFactor, Axis::X);
//cyls[cylCount]->SetCenter(Eigen::Vector3f(-0.8f * scaleFactor, 0, 0));
//cyls[0]->AddChild(cyls[cylCount]);
//cyls[cylCount]->isPickable = false;*/
//auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
//    return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
//};

//int snakeFoodAmount = 10;

//for (int i = 0; i < snakeFoodAmount; i++)
//{
//    snakeFood.push_back(Model::Create(std::to_string(i), meshCube, mat));
//    igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(snakeFood.at(i)->GetMesh()->data.front().vertices, snakeFood.at(i)->GetMesh()->data.front().faces);
//    food_kd_trees.push_back(kd_tree);
//    snakeFood.at(i)->Translate(Eigen::Vector3f(std::rand() % 12, 10, 0));
//    root->AddChild(snakeFood.at(i));
//    models.push_back(snakeFood.at(i));
//}

//dist = 0; // (snakeFood[0]->GetTranslation() - CalcTipPOs()).norm();
//root->AddChild(snakeFood[0]);


///*tree.init(snakeFood.front()->GetMesh()->data[0].vertices, snakeFood.front()->GetMesh()->data[0].faces);
//kd_trees.push_back(tree);*/


//// snake head collision box



//camera->RotateByDegree(60, Eigen::Vector3f(1, 0, 0));
////camera->RotateByDegree(-30, Axis::Z);

///*cube->mode = 1;
//auto mesh = cube->GetMeshList();*/

//int num_collapsed;

///*V = mesh[0]->data[0].vertices;
//F = mesh[0]->data[0].faces;
//igl::edge_flaps(F, E, EMAP, EF, EI);*/
///*std::cout << "vertices: \n" << V << std::endl;
//std::cout << "faces: \n" << F << std::endl;

//std::cout << "edges: \n" << E.transpose() << std::endl;
//std::cout << "edges to faces: \n" << EF.transpose() << std::endl;
//std::cout << "faces to edges: \n " << EMAP.transpose() << std::endl;
//std::cout << "edges indices: \n" << EI.transpose() << std::endl;*/

//// Added
//createJointBoxes();




//
////IGL_INLINE void BasicScene::drawAlignedBox(std::shared_ptr<cg3d::Model> cube, Eigen::AlignedBox<double, 3>& alignedBox) {
////
////
////
////    Eigen::MatrixXd V(8, 3);
////    Eigen::MatrixXi F(12, 3);
////    V.row(1) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomLeftFloor);
////    V.row(2) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor);
////    V.row(5) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor);
////    V.row(6) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopRightFloor);
////    V.row(0) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil);
////    V.row(3) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::BottomRightCeil);
////    V.row(4) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopLeftCeil);
////    V.row(7) = alignedBox.corner(Eigen::AlignedBox3d::CornerType::TopRightCeil);
////
////
////    F = cube->GetMesh()->data[0].faces;
////
////    Eigen::MatrixXd vertexNormals;
////
////    igl::per_vertex_normals(V, F, vertexNormals);
////
////    std::vector<cg3d::MeshData> newMeshDataList;
////    std::vector<std::shared_ptr<cg3d::Mesh>> newMeshList;
////    newMeshDataList.push_back({ V, F, vertexNormals, Eigen::MatrixXd::Zero(V.rows(), 2) });
////
////    newMeshList.push_back(std::make_shared<cg3d::Mesh>("new mesh", newMeshDataList));
////    cube->SetMeshList(newMeshList);
////
////    cube->showWireframe = true;
////    cube->showFaces = false;
////    //cube->isPickable = false;
////
////}
//void BasicScene::AddViewportCallback(Viewport* _viewport)
//{
//    viewport = _viewport;
//
//    Scene::AddViewportCallback(viewport);
//}
//void BasicScene::ViewportSizeCallback(Viewport* _viewport)
//{
//    for (auto& cam : camList)
//        cam->SetProjection(float(_viewport->width) / float(_viewport->height));
//
//    // note: we don't need to call Scene::ViewportSizeCallback since we are setting the projection of all the cameras
//}
//void BasicScene::LoadObjectFromFileDialog()
//{
//    std::string filename = igl::file_dialog_open();
//    if (filename.length() == 0) return;
//
//    //auto shape = Model::Create(filename, material);
//}
//
//
////void BasicScene::MoveDownObject(std::shared_ptr<cg3d::Model> model) {
////    int  i = 0;
////    while (i < 100000) {
////        model->Translate({ 0, -0.1f, 0 });
////    }
////
////}
//
//IGL_INLINE void BasicScene::move_snake() {
//    double snake_speed = 0.15;
//
//    if (!Playing || Paused || GameOver) {
//        position_offset = Eigen::Vector3d::Zero();
//        return;
//    }
//
//    if (direction != ' ')
//    {
//        switch (direction) {
//        case 'l':
//            position_offset = Eigen::Vector3d(0, 0, -snake_speed);
//            break;
//        case 'r':
//            position_offset = Eigen::Vector3d(0, 0, snake_speed);
//            break;
//        case 'u':
//            position_offset = Eigen::Vector3d(0, snake_speed, 0);
//            break;
//        case 'd':
//            position_offset = Eigen::Vector3d(0, -snake_speed, 0);
//            break;
//        default:
//            break;
//        }
//
//        vT[0] = skeleton[0];
//
//        for (int i = 0; i < cylCount; ++i) {
//            vT[i + 1] = skeleton[i + 1];
//            vT[i] += (vT[i + 1] - vT[i]) / 6;
//        }
//
//        vT[cylCount] += position_offset;
//        // W - weights matrix
//        // BE - Edges between joints
//        // C - joints positions
//        // P - parents
//        // M - weights per vertex per joint matrix
//        // U - new vertices position after skinning
//        
//        igl::dqs(V, W, vQ, vT, U);
//        /*Mesh m = Mesh("newSnake", U, models[0]->GetMesh()->data.front().faces, models[0]->GetMesh()->data.front().vertexNormals, models[0]->GetMesh()->data.front().textureCoords);
//        models[0]->SetMeshList(m);
//        */
//        for (int i = 0; i < split_snake.size(); ++i) {
//            Eigen::Vector3d pos_offset = vT[i] - skeleton[i];
//            Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(pos_offset.reverse(), skeleton[i].reverse());
//            
//            split_snake[i].Translate(Eigen::Vector3f(pos_offset[2], pos_offset[1], pos_offset[0]));
//            //Eigen::Vector4f v = Eigen::Vector4f(quat.coeffs());
//            split_snake[i].Rotate(quat.coeffs()[0], Eigen::Vector3f(quat.coeffs()[1], quat.coeffs()[2], quat.coeffs()[3]));
//        }
//        
//        for (int i = 0; i < skeleton.size(); ++i)
//            skeleton[i] = vT[i];
//
//        check_collision();
//    }
//}
//
//IGL_INLINE void BasicScene::createJointBoxes() {
//    double epsilon = 0.4;
//
//    for (int i = 1; i < cylCount + 1; ++i)
//    {
//        Eigen::Vector3d pos =  skeleton[i - 1];
//        Eigen::Vector3d m = pos + Eigen::Vector3d(-epsilon, -epsilon, -epsilon);
//        Eigen::Vector3d M = pos + Eigen::Vector3d(epsilon, epsilon, epsilon);
//        Eigen::AlignedBox<double, 3> boxforcurrJoint(m, M);
//        jointBoxes.push_back(boxforcurrJoint);
//    }
//}
//
//// start a new level
//IGL_INLINE void BasicScene::start_level() {
//
//    start_time = static_cast<int>(glfwGetTime());
//    prev_tic = static_cast<int>(glfwGetTime());
//    paused_time = 0;
//
//    p = 1.0 / level + 0.33;
//}
//
//IGL_INLINE void BasicScene::generate_target()
//{
//    if (!Playing || Paused || GameOver)
//        return;
//
//    if (level == 1 && level1_obj_amount > 3)
//        return;
//
//    float tic = static_cast<float>(glfwGetTime());
//
//    if (tic - prev_tic > creation_gap) {
//        prev_tic = tic;
//
//        std::this_thread::sleep_for(std::chrono::microseconds(5));
//        models.push_back(Model::Create("Food" + foodCunter, sphereMesh, mat));
//        igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(models.back()->GetMesh()->data.front().vertices, models.back()->GetMesh()->data.front().faces);
//        food_kd_trees.push_back(kd_tree);
//        //load_mesh_from_file("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/data/sphere.obj");
//        if (data_list.size() > parents.size())
//        {
//            parents.push_back(-1);
//            models.back()->showFaces = true;
//            /*data_list.back().set_visible(false, 1);
//            data_list.back().set_visible(true, 2);
//            data_list.back().show_faces = 3;*/
//        }
//
//        if (level == 1) {
//            update_movement_type(Model::BASIC, models.back());
//        }
//        else if (target2_creation == 0) { // generate different targets according to level
//            update_movement_type(Model::BEZIER, models.back());
//            target2_creation = 3;
//        }
//        else {
//            double target_proba = (double)(rand() % 10) / 10;
//
//            target_proba < p ? update_movement_type(Model::BASIC, models.back()) :
//                update_movement_type(Model::BOUNCY, models.back());
//
//            target2_creation--;
//        }
//
//        /*models.back()->type == BEZIER ? models.back()->set_colors(Eigen::RowVector3d(0, 0, 1)) :
//            models.back()->type == BOUNCY ? models.back()->set_colors(Eigen::RowVector3d(1, 0, 0)) :
//            data().set_colors(Eigen::RowVector3d(0, 1, 0));*/
//
//       initiate_speed(level1_obj_amount, models.back());
//        level1_obj_amount++;
//    }
//}
//
//void BasicScene::update_movement_type(int type, std::shared_ptr<cg3d::Model> model1 ) {
//    
//    switch (type) {
//        case 0:
//            model1->type = Model::NONE;
//            break;
//        case 1:
//            model1->type = Model::BASIC;
//            break;
//        case 2:
//            model1->type = Model::BOUNCY;
//            break;
//        case 3:
//            model1->type = Model::BEZIER;
//            break;
//        }
//}
//
//

//
//IGL_INLINE void BasicScene::move_targets()
//{
//    for (int i = cylCount + 1; i < models.size(); i++)
//        if (models[i]->type != Model::NONE) 
//            move(models[i]);
//}
//
//IGL_INLINE void BasicScene::move(std::shared_ptr<cg3d::Model> model1)
//{
//
//    if (model1->type == Model::BEZIER) {
//        double velocity = 0.5;
//        model1->t += 0.05 * velocity / 2;
//
//        if (model1->t <= 1) {
//            model1->T << powf(model1->t, 3), powf(model1->t, 2), model1->t, 1;
//            model1->curr_pos = model1->T * model1->MG;
//            //Eigen::Vector3d tangent = (model1->curr_pos - model1->last_pos).normalized();
//            ////model1->LookAt(tangent);
//            Eigen::Vector3f pos = Eigen::Vector3f{(float) model1->curr_pos[0],(float)model1->curr_pos[1] ,(float)model1->curr_pos[2] };
//            model1->Translate(pos);
//        }
//        else {
//            Eigen::Vector3f pos = Eigen::Vector3f{ (float)model1->final_dir[0],(float)model1->final_dir[1] ,(float)model1->final_dir[2] };
//            model1->Translate(pos * velocity * 0.05);
//        }
//        model1->last_pos = model1->curr_pos;
//    }
//    if (model1->type == Model::BOUNCY) {
//        Eigen::Vector3f speedf = Eigen::Vector3f{ (float)model1->speed[0],(float)model1->speed[1] ,(float)model1->speed[2] };
//        model1->TranslateInSystem(GetRotation(), speedf);
//
//        model1->speed.y() -= 0.05; // gravity
//
//        if (Tout.matrix()(1, 3) < -4) {
//           // PlaySound(TEXT("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/sounds/ballbounce.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
//            model1->speed.y() = -model1->speed.y();
//        }
//
//        // streching the ball
//        model1->speed.y() < 0 ? model1->Scale(Eigen::Vector3f(1, 1.05, 1)) :
//            model1->Scale(Eigen::Vector3f(1, 0.95, 1));
//
//    }
//    else {
//        Eigen::Vector3f speedf = Eigen::Vector3f{ (float)model1->speed[0],(float)model1->speed[1] ,(float)model1->speed[2] };
//        model1->TranslateInSystem(GetRotation(), speedf);
//    }
//
//
//}
//
//IGL_INLINE void BasicScene::check_level_up() {
//    if (score >= 40 * level) {
//        level++;
//        isLevelUp = true;
//        isActive = false;
//        score = 0;
//        timer = 0;
//
//        creation_gap = 2;
//
//        //PlaySound(TEXT("C:/Users/pijon/OneDrive/Desktop/animation3D/tutorial/sounds/nextLevel.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
//    }
//}