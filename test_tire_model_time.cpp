#include <iostream>
#include <chrono>
#include "tire_model.hpp"
#include "uttilities.hpp"
#include "ParamBank.hpp"

using namespace lem_dynamics_sim_;

int main() {
    // Inicjalizacja parametrów
    ParamBank P;
    P.add("m", 1500.0);
    P.add("gravity", 9.81);
    P.add("w", 1.6);
    P.add("a", 0.4);
    P.add("b", 1.2);
    P.add("t_front", 1.5);
    P.add("t_rear", 1.5);
    P.add("h", 0.5);
    P.add("h_roll_f", 0.3);
    P.add("h_roll_r", 0.4);
    P.add("pCx1", 1.2);
    P.add("pEx1", 0.8);
    P.add("pDx1", 1.1);
    P.add("pDx2", 0.9);
    P.add("pKx1", 1.0);
    P.add("pKx3", 0.7);
    P.add("lambda_x", 0.95);
    P.add("pCy1", 1.3);
    P.add("pEy1", 0.85);
    P.add("pDy1", 1.2);
    P.add("pDy2", 1.0);
    P.add("pKy1", 1.1);
    P.add("pKy2", 0.9);
    P.add("lambda_y", 0.92);
    P.add("N0", 1000.0);
    P.add("epsilon", 0.05);
    P.add("cx", 0.4);
    P.add("dx", 0.1);
    P.add("cy", 0.3);
    P.add("dy", 0.2);
    P.add("r_rear", 0.35);
    P.add("r_front", 0.35);
    P.add("angle_construction_front", 0.1);
    P.add("angle_construction_rear", 0.1);
    P.add("R", 0.3);
    P.add("Kf", 30000.0);  // Dodano brakujący klucz
    P.add("Kr", 30000.0);  // Dodano brakujący klucz
    P.add("Cl1", 0.2);
    P.add("Cl2", 0.1);

    // Inicjalizacja stanu
    State x;
    x.x = 0.0;
    x.y = 0.0;
    x.yaw = 0.0;
    x.vx = 10.0;  // Prędkość wzdłużna
    x.vy = 2.0;   // Prędkość boczna
    x.yaw_rate = 0.1;
    x.omega_rr = 25.0;
    x.omega_rl = 29.0;
    x.delta_left = 0.1;
    x.delta_right = 0.1;

    // Inicjalizacja wejścia
    Input u;
    u.torque_request = 100.0;
    u.rack_angle_request = 0.1;

    // Pomiar czasu wykonania
    auto start = std::chrono::high_resolution_clock::now();
    State result = derative_tire_model(P, x, u);
    auto end = std::chrono::high_resolution_clock::now();

    // Wyświetlenie wyników
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Czas wykonania: " << elapsed.count() << " sekund" << std::endl;

    // Wyświetlenie wynikowego stanu
    std::cout << "Wynikowy stan:" << std::endl;
    std::cout << "fx_rr: " << result.fx_rr << ", fy_rr: " << result.fy_rr << std::endl;
    std::cout << "fx_rl: " << result.fx_rl << ", fy_rl: " << result.fy_rl << std::endl;
    std::cout << "fy_fl: " << result.fy_fl << ", fy_fr: " << result.fy_fr << std::endl;

    return 0;
}