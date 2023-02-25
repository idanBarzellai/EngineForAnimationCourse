#pragma once


#include "AutoMorphingModel.h"
#include "Scene.h"
#include "DataStructure.h"

#include "SceneWithImGui.h"

#include <memory>
#include <utility>
#include <igl/AABB.h>

#include <stack>
class BasicScene : public cg3d::SceneWithImGui
{
    enum GameState {Playing, Paused, GameOver};
public:
    BasicScene(std::string name, cg3d::Display* display);
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    bool boxes_collide(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2, Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox);
    bool treeNodesCollide(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2, igl::AABB<Eigen::MatrixXd, 3>& firstObjNode, igl::AABB<Eigen::MatrixXd, 3>& secondObjNode);
    bool check_collision();
    std::vector < igl::AABB<Eigen::MatrixXd, 3>> kd_trees;
    void initiate_speed(int objAmount, std::shared_ptr<cg3d::Model> model1);
    std::vector< std::shared_ptr<cg3d::Model>> models, targets, cyls;



private:


    typedef
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
        RotationList;

    // W - weights matrix
    // BE - Edges between joints
    // C - joints positions
    // P - parents
    // M - weights per vertex per joint matrix
    // U - new vertices position after skinning
 /*   Eigen::MatrixXd V, W, C, U, M;
    Eigen::MatrixXi F, BE;
    Eigen::VectorXi P;*/
    //std::vector<RotationList > poses; // rotations of joints for animation
    std::vector<Eigen::Vector3d> calc_vT();
    RotationList vQ;
    std::vector<Eigen::Vector3d> vT;
    Eigen::MatrixXd V, W, U;

    std::vector< Eigen::MatrixXd> weightsVec;
    float scaleFactor;
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Material> mat;
    std::shared_ptr<cg3d::Material> mat2;

    std::shared_ptr<cg3d::Model> snake;
    std::shared_ptr<cg3d::Mesh> meshCube;
    void Animate();
    void MoveTarget(bool changesides);
    void BuildImGui() override;
    void SetLevel(int level);
    void resetSnake();
    void PlayAudio(char* audio);
    void AddOrLosePoint(bool isAdd);
    void Skinning();
    void LinearSkinning();
    void UpdateMesh(Eigen::MatrixXd U);
    Eigen::MatrixXd weigths_calc(int bonesNum);
    RotationList calc_Vq();
    std::vector<std::shared_ptr<cg3d::Camera>> camList{ 4 };
    std::map<std::shared_ptr<cg3d::Model>, igl::AABB<Eigen::MatrixXd, 3>> kd_trreModelsMap;

    bool isSnakeHeadCamera = false;

    bool isActive;
    int cylCount;
    int cylIndex = 0;
    float dist;
    float delta;

    float max_dis;

    float timer = 0;
    float generatingTime = 0.5f;
    float lastTimeGenerated = 0;

    float gameOverTime = 30;
    float lastGameOverTime = 0;

    int score =0;
    int level = 0;
    int goalScore = 3;
    bool soundOn = true;
    int currAmountOfObjs = 0;
    int totalObjAmount = 10;
    bool changeSidesNow = false;

    float baseSpeed = 0.006f;
    float speed = 0.006f;
    float radius = 0.1f;

    int levelProperties = 50;

    float changeSIdesTime = 0;
    std::vector<Eigen::Vector3f> targetMovementPos;
    std::vector<int> randomPlacesX;
    std::vector <bool> targetSides;
};
