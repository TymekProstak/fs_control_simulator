 
 #include "engine.h"
 
 namespace metzler_model {
 
    Engine::Engine(const engine_params& params, const engine_state& initial_state )
            : params_(params), state_(initial_state) {}

    double Engine::engine_torque_derivative(double engine_input) const {

        // Calculate the derivative of the actual torque based on engine parameters and input 
        // assumed model is first order system -> more complex models can be implemented here

        double torque_derivative =  (params_.max_torque * engine_input - state_.torque)/ params_.charactersistic_time;

        return torque_derivative;

    }

    void Engine::update_torque(double engine_input, double dt) {
        // Update the torque of the engine based on the input and time step
        // This is a simple first-order system update, can be modified for more complex dynamics

        double derivative = engine_torque_derivative(engine_input);
        state_.torque += derivative * dt;

        // Low/Hgh filter can be added here to limit the torque within the min/max bounds 

    }






 }