#ifndef ENGINE_H
#define ENGINE_H

#include "libs.h"

namespace metzler_model {

    struct engine_state {
        double torque;
    };

    /// **** //// strukutura z parametrami silnika //////
    struct engine_params {
        double charactersistic_time;
        double max_torque;
        double min_torque;
    };

    ///******//////////

    // Funkcja pochodnej momentu silnika
    inline double  engine_torque_derivative(const engine_state& state, const engine_params& params, const double engine_input) {
        double result;
        result= (params.max_torque * engine_input - state.torque) / params.charactersistic_time;
        return result;
    }

    class Engine {
    public:
        Engine(const engine_params& params, const engine_state& initial_state = engine_state{0.0})
            : params_(params), state_(initial_state) {}

        

        
        inline engine_state get_state() const { return state_; }

        inline void set_state(const engine_state& new_state) {
            state_ = new_state;
        }

        inline double get_torque() const {
            return state_.torque;
        }

        inline void update( double engine_input, double dt) {
            // Calculate the derivative of the torque
            double derivative = engine_torque_derivative(state_, params_, engine_input);
            // Update the state using Euler's method -> will be used in simulation as a step of RK4
            state_.torque += derivative * dt;
        }

        inline void reset(const engine_state& initial_state = engine_state{0.0}) {
            state_ = initial_state;
        }

        inline void set_params(const engine_params& new_params) {
            params_ = new_params;
        }


    private:
        engine_params params_;
        engine_state state_;
    };

    

} // namespace metzler_model

#endif