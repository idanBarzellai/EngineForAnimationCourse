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
    //enum type { NONE, BASIC, BOUNCY, BEZIER };
public:
    BasicScene(std::string name, cg3d::Display* display);
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    //IGL_INLINE void FABRIK(Eigen::Vector3f dest);
    //Eigen::Vector3f CalcTipPOs();
    //IGL_INLINE Eigen::Matrix3f CalcParentsInverseRotation(int index);
    //void fixAxis();
    bool boxes_collide(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2, Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox);
    bool treeNodesCollide(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2, igl::AABB<Eigen::MatrixXd, 3>& firstObjNode, igl::AABB<Eigen::MatrixXd, 3>& secondObjNode);
    //IGL_INLINE void drawAlignedBox(std::shared_ptr<cg3d::Model> cube, Eigen::AlignedBox<double, 3>& alignedBox);
    bool check_collision();
    std::vector < igl::AABB<Eigen::MatrixXd, 3>> kd_trees;
    //std::vector < igl::AABB<Eigen::MatrixXd, 3>> food_kd_trees;
    //void AddViewportCallback(cg3d::Viewport* _viewport) override;
    //void ViewportSizeCallback(cg3d::Viewport* _viewport) override;
    //void MoveDownObject(std::shared_ptr<cg3d::Model> model);

    //IGL_INLINE void move_snake();
    //IGL_INLINE void createJointBoxes();
    //IGL_INLINE void start_level();
    //IGL_INLINE void generate_target();
    //Eigen::Vector3d position_offset;
    //unsigned char direction;
    //unsigned char previous_direction;
    //int start_time = 0;
    //float prev_tic = 0;
    //int paused_time = 0 ;
    //int level1_obj_amount = 0;
    //int creation_gap = 0;
    //double snake_tail_start= -0.8;
    //double link_length = 1.6;
    //std::vector<int> parents;
    //int target2_creation = 2;
    //bool isLevelUp = false;
    //void update_movement_type(int type, std::shared_ptr<cg3d::Model> model1);
    void initiate_speed(int objAmount, std::shared_ptr<cg3d::Model> model1);
    //IGL_INLINE void move_targets();
    //IGL_INLINE void move(std::shared_ptr<cg3d::Model> model1);
    //IGL_INLINE void check_level_up();
    //double p; // probability to generate target of type 1
    //typedef
    //    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
    //    RotationList;

    //Eigen::MatrixXd V, W, C, U, M;
    //Eigen::MatrixXi F, BE;
    //Eigen::VectorXi P;
    //RotationList vQ;
    //std::shared_ptr<cg3d::Mesh> sphereMesh;


    //std::vector<Eigen::AlignedBox<double, 3>> jointBoxes;
    //
    //
    //std::vector<Eigen::Vector3d> vT;
    //std::vector<Eigen::Vector3d> skeleton;
    //std::vector<Movable> split_snake;
    ////Eigen::MatrixXd V1; // Vertices of the current mesh (#V x 3)
    ////Eigen::MatrixXi F1; // Faces of the mesh (#F x 3)

    ////std::vector < igl::AABB<Eigen::MatrixXd, 3>> kd_trees;
    ////Eigen::MatrixXd V2; // Vertices of the current mesh (#V x 3)
    ////Eigen::MatrixXi F2; // Faces of the mesh (#F x 3)
    std::vector< std::shared_ptr<cg3d::Model>> models, targets, cyls;
    //Eigen::MatrixXd points1;
    //uint32_t dirty;


private:



    float scaleFactor;
    std::shared_ptr<Movable> root;
    //std::shared_ptr<cg3d::Model> snakeFoodFirst, cube, cube2, cubeBox, cubeBox2;
    std::shared_ptr<cg3d::Material> mat;
    std::shared_ptr<cg3d::Material> mat2;

    std::shared_ptr<cg3d::Mesh> meshCube;

    //std::unordered_map<std::string, std::vector<std::shared_ptr<DataStructure>>> dataStructures;
    //std::map<std::string, std::stack<std::vector<std::shared_ptr<cg3d::Mesh>>>> originalMeshes;
    //std::vector<std::shared_ptr<cg3d::Model>> snakeFood;
    //void createDataStructures(std::shared_ptr<cg3d::Model> model);
    //void simplify();
    //void resetMesh();
    void Animate();
    void MoveTarget(bool changesides);
    void BuildImGui() override;
    void SetLevel(int level);
    void resetSnake();
    //void LoadObjectFromFileDialog();
    std::vector<std::shared_ptr<cg3d::Camera>> camList{ 4 };
    //cg3d::Viewport* viewport = nullptr;
    std::map<std::shared_ptr<cg3d::Model>, igl::AABB<Eigen::MatrixXd, 3>> kd_trreModelsMap;
    //std::shared_ptr<cg3d::AutoMorphingModel> autoCube;
    //std::vector<std::shared_ptr<cg3d::Model>>cyls;
    //int pickedIndex = 0;
    //int tipIndex = 0;
    ///*Eigen::VectorXi EMAP;
    //Eigen::MatrixXi F, E, EF, EI;
    //Eigen::VectorXi EQ;
    //Eigen::MatrixXd V, C, N, T, points, edges, colors;*/
    bool isSnakeHeadCamera = false;

    bool isActive;
    int cylCount;
    int cylIndex = 0;
    float dist;
    float delta;
    ////float cyl_length = 1.6f;
    //Eigen::RowVector4f destination;
    float max_dis;

    float timer = 0;
    float generatingTime = 0.5f;
    float lastTimeGenerated = 0;

    float gameOverTime = 3;
    float lastGameOverTime = 0;

    //int endLevelAmount = 3;
    //int foodCunter = 0;
    int score =0;
    int level = 0;
    //bool isSnakeHead = false;
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
    std::vector <bool> targetSides;
};
