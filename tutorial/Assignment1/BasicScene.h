#include "Scene.h"
#include "DataStructure.h"
#include <igl/AABB.h>

#include <utility>
#include <stack>
class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    bool isActive;
    bool boxes_collide(Eigen::AlignedBox<double, 3>& firstbox, Eigen::AlignedBox<double, 3>& secondbox);
    bool treeNodesCollide(igl::AABB<Eigen::MatrixXd, 3>& firstObjNode, igl::AABB<Eigen::MatrixXd, 3>& secondObjNode);
    IGL_INLINE void drawAlignedBox(std::shared_ptr<cg3d::Model> cube, Eigen::AlignedBox<double, 3>& alignedBox);
    bool check_collision();
    double findFarestVertexFromCenter(std::shared_ptr<cg3d::Model> model);

    igl::AABB<Eigen::MatrixXd, 3> kd_tree;
    Eigen::MatrixXd V; // Vertices of the current mesh (#V x 3)
    Eigen::MatrixXi F; // Faces of the mesh (#F x 3)

    igl::AABB<Eigen::MatrixXd, 3> kd_tree2;
    Eigen::MatrixXd V2; // Vertices of the current mesh (#V x 3)
    Eigen::MatrixXi F2; // Faces of the mesh (#F x 3)
    std::vector< std::shared_ptr<cg3d::Model>> models;
    Eigen::MatrixXd points;
    uint32_t dirty;



private:
    double scaleFactor;
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cube, cube2, cubeBox, cubeBox2;
    std::shared_ptr<cg3d::Material> mat;
    std::unordered_map<std::string, std::vector<std::shared_ptr<DataStructure>>> dataStructures;
    std::map<std::string, std::stack<std::vector<std::shared_ptr<cg3d::Mesh>>>> originalMeshes;

    void createDataStructures(std::shared_ptr<cg3d::Model> model);
    void simplify();
    void resetMesh();
    void Animate();

};