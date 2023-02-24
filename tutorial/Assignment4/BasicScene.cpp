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
        ImGui::Text("Score: %.2f", timer);

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
    auto carbon{ std::make_shared<Material>("carbon", program1) };

    grass->AddTexture(0, "textures/grass.bmp", 2);
    
    mat->AddTexture(0, "textures/box0.bmp", 2);
    mat2->AddTexture(1, "textures/snake.jpg", 2);
    carbon->AddTexture(0, "textures/carbon.jpg", 2);


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


    models.push_back(Model::Create("floor", meshCube, grass));
    root->AddChild(models[0]);
    models[0]->Scale(levelProperties - 10);
    models[0]->Translate(Eigen::Vector3f(0, 0, -levelProperties/2));
    models[0]->isPickable = false;


    for (int i = 0; i < cylCount; i++)
    {
        cyls.push_back(Model::Create("cyls" + i, cylMesh, mat));
        i == 0 ? root->AddChild(cyls[i]) : cyls[i - 1]->AddChild(cyls[i]);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::X);
        cyls[i]->SetCenter(Eigen::Vector3f(-0.8f * scaleFactor, 0, 0));
        cyls[i]->isPickable = false;

        if (i == 0) {
            igl::AABB<Eigen::MatrixXd, 3> kd_tree; kd_tree.init(cyls[i]->GetMesh()->data[0].vertices, cyls[i]->GetMesh()->data[0].faces);
            kd_trreModelsMap.emplace(cyls[i], kd_tree);
        }
        
    }
    for (int i = 4; i < cylCount; i++)
    {
        cyls[i]->isHidden = true;
    }
    for (int i = 0; i < totalObjAmount; i++)
    {
        targets.push_back(Model::Create("newTarget" + i, meshCube, i% 2 ?  mat : carbon));
        root->AddChild(targets[i]);
        targets[i]->Translate(Eigen::Vector3f(randomPlacesX[i % 9], randomPlacesY[i % 9], 0));
        targets[i]->isHidden = true;
        targets[i]->isPickable = false;
        targetSides.push_back(false);
        


    }
    system("CLS");
    std::string answer;
    std::cout << "Welcome to 'Snake - UPGRADED!'\nGet point to improve the snake!\n\nRules:\n1) Red box = 1 point\n2) Black box = -1 point\n3) Collect 3 points to level up\n4) Stay inside the green box with all the snake to stay alive\n5) When leveling up the bounderies get smaller\n6) You have 30 seconds to get 3 point otherwise you lose\n\nDo you want to start? Y/N: ";
    std::cin >> answer; // get user input from the keyboard
    if (!answer.compare("Y") || !answer.compare("y")) {
        std::cout << "Lets go!: " << std::endl;
        if (soundOn)
            PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/game.wav"), NULL, SND_ASYNC);
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
        if (soundOn)
            PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/addingball.wav"), NULL, SND_ASYNC);
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
    
    // Checking if time is up
    if (timer  - lastGameOverTime > gameOverTime) {
        
        if(soundOn)
            PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/end_notice.wav"), NULL, SND_ASYNC);
        system("CLS");

        std::string answer;
        std::cout << "Do you want to continue the Game? (Y/N): ";
        std::cin >> answer; // get user input from the keyboard
        if (!answer.compare("Y") || !answer.compare("y")) {
            std::cout << "Lets go!: " << std::endl;
            SetLevel(0);
        }
        else {
            std::cout << "OK bye... \nQuitting" << std::endl;
            exit(0);
        }
        lastGameOverTime = timer;
    }

    
    // Checking for end game
    if (score == goalScore) {
        SetLevel(++level);
    }
    
    // Getting Point
    for (int i = 0; i < targets.size(); i++)
    {
        Eigen::Vector3f dist = targets[i]->GetTranslation() - cyls[0]->GetTranslation();
        if (sqrt((dist[0] * dist[0]) + (dist[1] * dist[1]) + (dist[2] * dist[2])) < 2) {
            check_collision()
        }
    }

    // Losing game
    for (int i = 0; i < 4 + score; i++)
    {
        float boundries = (levelProperties - 10) / 2;
        bool outOfBounderied = cyls[i]->GetTranslation()[0] >= boundries || cyls[i]->GetTranslation()[0] <= -boundries || cyls[i]->GetTranslation()[1] >= boundries || cyls[i]->GetTranslation()[1] <= -boundries;

        if (outOfBounderied || score < -2) {
            if (soundOn)
                PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/LostGame.wav"), NULL, SND_ASYNC);
            system("CLS");

            std::string answer;
            std::cout << "You LOST! Thanks for playing!\nPress any key and Enter to exit... ";
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
    score = 0;

    if (level == 0) {
        levelProperties = 50;
        for (int i = 0; i < 4; i++)
        {
            cyls[i]->isHidden = false;
        }
    }
    else {
        levelProperties = levelProperties - 5;
        camera->Translate(-5, Axis::Z);
        models[0]->Scale(0.9);
    }
    resetSnake();
   
}

//void BasicScene::PlayAudio(char * audio) {
//    LPCSTR audioPath = "C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/" + audio;
//    if (soundOn)
//        PlaySound(TEXT(audioPath), NULL, SND_ASYNC);
//}

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
            /*cyls[0]->RotateInSystem(system, speed / radius, Axis::Y);
            cyls[1]->RotateInSystem(system, -speed / radius, Axis::Y);
            for (int i = 2; i < cylCount; i++)
                cyls[i]->RotateInSystem(system, -speed / radius, Axis::Y);*/
            break;
        case GLFW_KEY_RIGHT:
           /* cyls[0]->RotateInSystem(system, -speed / radius, Axis::Y);
            cyls[1]->RotateInSystem(system, speed / radius, Axis::Y);
            for (int i = 2; i < cylCount; i++)
                cyls[i]->RotateInSystem(system, speed / radius, Axis::Y);*/

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

IGL_INLINE bool BasicScene::check_collision() {
    if (kd_trreModelsMap.size() < 2)
        return false;
    for (int i = 0; i < kd_trreModelsMap.size() -1; i++) {
        std::shared_ptr<cg3d::Model> currModel = targets[i];
        igl::AABB<Eigen::MatrixXd, 3> currKd_tree = kd_trreModelsMap.at(targets[i]); //kd_trees[i + 1];
        if (treeNodesCollide(cyls[0], currModel, kd_trreModelsMap.at(cyls[0]), currKd_tree)) {
            //score++;
            currModel->isHidden = true;
            kd_trreModelsMap.at(targets[i]).deinit();
            
            //cyls[3 + score]->isHidden = false;
            AddOrLosePoint(i % 2);
            return true;
        }
    }
    return false;
}

void BasicScene::AddOrLosePoint(bool isAdd) {
    score = isAdd ? score + 1 : score - 1;
    cyls[3 + score]->isHidden = isAdd ? false : true;
    system("CLS");
    if (soundOn &&isAdd)
        PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/point.wav"), NULL, SND_ASYNC);
    else if(soundOn && !isAdd)
        PlaySound(TEXT("C:/Users/IdanBarzellai/Desktop/animation course/EngineForAnimationCourse/tutorial/sounds/lose.wav"), NULL, SND_ASYNC);

    std::string output = isAdd ? "+ 1 !!!!" : "- 1 ...";
    std::cout << output << std::endl;
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
