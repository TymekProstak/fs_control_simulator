#include"ParamBank.hpp"
#include"uttilities.hpp"

namespace lem_dynamics_sim{
    
    State derative_tire_model( const  ParamBank& P, const State& x, const Input& u) ;

    // czacik żróżniczkował symbolicznie MF 5.2  elgeancko , brawo panie czatgpt

    inline double dFy_dslipY(
        double slipx, double slipy,
        double N, double Dy, double Cy, double By, double Ey)
    {
        const double eps = 1e-12; // regularizacja dla r ~ 0
        const double r = std::hypot(slipx, slipy);
        const double r_safe = (r < eps) ? eps : r;
    
        const double inv_r   = 1.0 / r_safe;
        const double inv_r3  = inv_r * inv_r * inv_r;
    
        const double h = By * r_safe;
        const double g = h - Ey * (h - std::atan(h));
        const double A = Cy * std::atan(g);
    
        const double q       = slipy * inv_r;                 // slipy / r
        const double dq_dsy  = (slipx*slipx) * inv_r3;        // slipx^2 / r^3
    
        // dA/dslipy:
        // term = By*(slipy/r) * (1 + (1 - Ey)*h^2) / (1 + h^2)
        const double num_h   = 1.0 + (1.0 - Ey) * h * h;
        const double den_h   = 1.0 + h * h;
        const double term    = By * (slipy * inv_r) * (num_h / den_h);
        const double dA_dsy  = Cy * (term / (1.0 + g * g));
    
        return N * Dy * ( dq_dsy * std::sin(A) + q * std::cos(A) * dA_dsy );
    }

    // czacik żróżnicował symbolicznie MF 5.2  elgeancko , brawo panie czatgpt

    inline double dFx_dslipX(
        double slipx, double slipy,
        double N, double Dx, double Cx, double Bx, double Ex)
    {
       
    
        const double inv_r   = 1.0 / r_safe;
        const double inv_r3  = inv_r * inv_r * inv_r;
    
        const double h = Bx * r_safe;
        const double g = h - Ex * (h - std::atan(h));
        const double A = Cx * std::atan(g);
    
        const double q       = slipx * inv_r;                 // slipy / r
        const double dq_dsx  = (slipy*slipy) * inv_r3;       // -slipy^2 / r^3
    
        // dA/dslipx:
        // term = By*(slipx/r) * (1 + (1 - Ey)*h^2) / (1 + h^2)
        const double num_h   = 1.0 + (1.0 - Ex) * h * h;
        const double den_h   = 1.0 + h * h;
        const double term    = Bx * (slipx * inv_r) * (num_h / den_h);
        const double dA_dsx  = Cx * (term / (1.0 + g * g));
    
        return N * Dx * ( dq_dsx * std::sin(A) + q * std::cos(A) * dA_dsx );
    }
}