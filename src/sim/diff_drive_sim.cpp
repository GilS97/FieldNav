// Simulation MuJoCo du robot (LAFVIN 4WD) : chargement, pas de physique, lecture de l'état.
#include <mujoco/mujoco.h>

#include <cmath>
#include <fieldnav/sim/diff_drive_sim.hpp>
#include <stdexcept>

namespace fieldnav::sim {

namespace {

int requireId(const mjModel* m, mjtObj type, const char* name) {
    const int id = mj_name2id(m, type, name);
    if (id < 0) {
        throw std::runtime_error(std::string("robot_diffdrive : élément MuJoCo manquant : ") +
                                 name);
    }
    return id;
}

int wheelDof(const mjModel* m, const char* joint_name) {
    return m->jnt_dofadr[requireId(m, mjOBJ_JOINT, joint_name)];
}

} // namespace

DiffDriveSim::DiffDriveSim(const std::string& model_path) {
    char error[1000] = "";
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (model_ == nullptr) {
        throw std::runtime_error("Chargement de " + model_path + " impossible : " + error);
    }
    // Les noms viennent de scenes/robot_diffdrive.xml. Un nom manquant libère le modèle avant
    // de propager l'erreur (le destructeur n'est pas appelé si le constructeur lance).
    try {
        qpos_root_ = model_->jnt_qposadr[requireId(model_, mjOBJ_JOINT, "root")];
        dof_fl_ = wheelDof(model_, "j_wheel_fl");
        dof_rl_ = wheelDof(model_, "j_wheel_rl");
        dof_fr_ = wheelDof(model_, "j_wheel_fr");
        dof_rr_ = wheelDof(model_, "j_wheel_rr");
        act_fl_ = requireId(model_, mjOBJ_ACTUATOR, "m_fl");
        act_rl_ = requireId(model_, mjOBJ_ACTUATOR, "m_rl");
        act_fr_ = requireId(model_, mjOBJ_ACTUATOR, "m_fr");
        act_rr_ = requireId(model_, mjOBJ_ACTUATOR, "m_rr");

        // Rayon : size[0] du pneu. Voie : écart en y entre les roues avant gauche et droite.
        wheel_radius_ = model_->geom_size[3 * requireId(model_, mjOBJ_GEOM, "tire_fl")];
        const int bl = requireId(model_, mjOBJ_BODY, "wheel_fl");
        const int br = requireId(model_, mjOBJ_BODY, "wheel_fr");
        track_width_ = std::abs(model_->body_pos[3 * bl + 1] - model_->body_pos[3 * br + 1]);
    } catch (...) {
        mj_deleteModel(model_);
        throw;
    }
    data_ = mj_makeData(model_);

    // Pose initiale cohérente (positions des corps calculées avant le premier pas).
    mj_forward(model_, data_);
}

DiffDriveSim::~DiffDriveSim() {
    mj_deleteData(data_);
    mj_deleteModel(model_);
}

void DiffDriveSim::setWheelSpeeds(const WheelSpeeds& target) noexcept {
    // Comme sur le shield : les deux moteurs d'un côté reçoivent la même consigne.
    data_->ctrl[act_fl_] = target.left;
    data_->ctrl[act_rl_] = target.left;
    data_->ctrl[act_fr_] = target.right;
    data_->ctrl[act_rr_] = target.right;
}

void DiffDriveSim::step() noexcept {
    mj_step(model_, data_);
}

void DiffDriveSim::reset() noexcept {
    mj_resetData(model_, data_);
    mj_forward(model_, data_);
}

double DiffDriveSim::time() const noexcept {
    return data_->time;
}

double DiffDriveSim::timestep() const noexcept {
    return model_->opt.timestep;
}

odometry::Pose2D DiffDriveSim::truePose() const noexcept {
    // freejoint : x, y, z puis quaternion (qw, qx, qy, qz) ; cap = rotation autour de z.
    const double* q = data_->qpos + qpos_root_;
    const double qw = q[3], qx = q[4], qy = q[5], qz = q[6];
    const double theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    return odometry::Pose2D{q[0], q[1], theta};
}

WheelSpeeds DiffDriveSim::wheelSpeeds() const noexcept {
    // Vitesse d'un côté = moyenne de ses deux roues (elles glissent un peu différemment en
    // virage) ; c'est ce que mesurerait un encodeur par côté.
    return WheelSpeeds{0.5 * (data_->qvel[dof_fl_] + data_->qvel[dof_rl_]),
                       0.5 * (data_->qvel[dof_fr_] + data_->qvel[dof_rr_])};
}

} // namespace fieldnav::sim
