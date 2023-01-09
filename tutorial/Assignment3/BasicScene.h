#pragma once
#include "AutoMorphingModel.h"
#include "Scene.h"

#include <memory>
#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    IGL_INLINE void IKSolver();
    Eigen::Vector3f CalcTipPOs();
    IGL_INLINE Eigen::Matrix3f CalcParentsInverseRotation(int index);
    void fixAxis();
private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> sphere1, cube;
    std::shared_ptr<cg3d::AutoMorphingModel> autoCube;
    std::vector<std::shared_ptr<cg3d::Model>>models, cyls, axis;
    int pickedIndex = 0;
    int tipIndex = 0;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C, N, T, points, edges, colors;
    bool isActive;
    void Animate();
    int cylCount;
    float dist;
    float delta;
    float cyl_length = 1.6f;
    Eigen::RowVector4f destination;
    float max_dis;
    float scaleFactor;
};
