#include <iostream>
#include <Eigen/Dense>
#include <vector> // Dodano dla std::vector
#include <functional> // Dodano dla std::function

namespace lem_dynamics_sim_ {

// Dziedziczenie z Eigen::Matrix dla pełnej kompatybilności
struct State : public Eigen::Matrix<double, 17, 1> {
    // Konstruktor domyślny
    State() : Eigen::Matrix<double, 17, 1>() {}

    // Konstruktor inicjalizujący wszystkie elementy zerami
    explicit State(double value) : Eigen::Matrix<double, 17, 1>() {
        this->setConstant(value);
    }

    // Konstruktor inicjalizujący z std::vector
    explicit State(const std::vector<double>& values) : Eigen::Matrix<double, 17, 1>() {
        if (values.size() != 17) {
            throw std::invalid_argument("Vector size must be 17 to initialize State.");
        }
        for (std::size_t i = 0; i < 17; ++i) {
            (*this)(i) = values[i];
        }
    }

    // Dostęp do elementów przez nazwy

    // kinematic states
    double& x() { return (*this)(0); }
    double& y() { return (*this)(1); }
    double& yaw() { return (*this)(2); }
    double& vx() { return (*this)(3); }
    double& vy() { return (*this)(4); }
    double& yaw_rate() { return (*this)(5); }
    // wheels states
    double& omega_rr() { return (*this)(6); }
    double& omega_rl() { return (*this)(7); }
    // steering states
    double& delta_left(){ return (*this)(8);}
    double& d_delta_left(){ return (*this)(8);}
    double& delta_rigth(){ return(*this)(9);}
    double& d_delta_rigth(){ return(*this)(9);}
    double& rack_angle() { return (*this)(10); }
    double& d_rack_angle() { return (*this)(11); }
    // drivetrain states
    double& torque() { return(*this)(12);}
    double& torque_left() const { return (*this)(13); }
    double& torque_right() const { return (*this)(14);}
    // tire model states for 1st order lag in force
    double& fx_rr() { return (*this)(15); }
    double& fx_rl() { return (*this)(16); }
    double& fy_rr() { return (*this)(17); }
    double& fy_rl() { return (*this)(18); }
    double& fy_fr() { return (*this)(19); }
    double& fy_fl() { return (*this)(20); }
    // previous acceleration for mass transfer model
    double& prev_ax() const { return (*this)(21); }
    double& prev_ay() const{return (*this)(22);}

    // Stały dostęp do elementów przez nazwy
    const double& x() { return (*this)(0); }
    const double& y() { return (*this)(1); }
    const double& yaw() { return (*this)(2); }
    const double& vx() { return (*this)(3); }
    const double& vy() { return (*this)(4); }
    const double& yaw_rate() { return (*this)(5); }
    const double& omega_rr() { return (*this)(6); }
    const double& omega_rl() { return (*this)(7); }
    const double& delta_left(){ return (*this)(8);}
    const double& d_delta_left(){ return (*this)(8);}
    const double& delta_rigth(){ return(*this)(9);}
    const double& d_delta_rigth(){ return(*this)(9);}
    const double& rack_angle() { return (*this)(10); }
    const double& d_rack_angle() { return (*this)(11); }
    const double& torque() { return(*this)(12);}
    const double& torque_left() const { return (*this)(13); }
    const double& torque_right() const { return (*this)(14);}
    const double& prev_ax() const { return (*this)(15); }
    const double& prev_ay() const{return (*this)(16);}

};

// Dziedziczenie z Eigen::Matrix dla Input
struct Input : public Eigen::Matrix<double, 2, 1> {
    // Konstruktor domyślny
    Input() : Eigen::Matrix<double, 2, 1>() {}

    // Konstruktor inicjalizujący wszystkie elementy zerami
    explicit Input(double value) : Eigen::Matrix<double, 2, 1>() {
        this->setConstant(value);
    }

    // Konstruktor inicjalizujący z std::vector
    explicit Input(const std::vector<double>& values) : Eigen::Matrix<double, 2, 1>() {
        if (values.size() != 2) {
            throw std::invalid_argument("Vector size must be 2 to initialize Input.");
        }
        for (std::size_t i = 0; i < 2; ++i) {
            (*this)(i) = values[i];
        }
    }

    // Dostęp do elementów przez nazwy
    double& torque_request() { return (*this)(0); }
    double& rack_angle_request() { return (*this)(1); }

    // Stały dostęp do elementów przez nazwy
    const double& torque_request() const { return (*this)(0); }
    const double& rack_angle_request() const { return (*this)(1); }
};

// Funkcja RK4 (Runge-Kutta 4. rzędu)
void RK4(
    const Input& u, 
    State& x, 
    const std::function<Eigen::Matrix<double, 17, 1>(const State&, const Input&)>& f, 
    double dt
) {
    // Obliczanie współczynników k1, k2, k3, k4
    Eigen::Matrix<double, 17, 1> k1 = f(x, u);
    Eigen::Matrix<double, 17, 1> k2 = f(x + 0.5 * dt * k1, u);
    Eigen::Matrix<double, 17, 1> k3 = f(x + 0.5 * dt * k2, u);
    Eigen::Matrix<double, 17, 1> k4 = f(x + dt * k3, u);

    // Aktualizacja stanu
    x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}



} // namespace lem_dynamics_sim_