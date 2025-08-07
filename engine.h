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



    class Engine {
    public:

        /// Constructor for the Engine class
        /// @param params Engine parameters
        /// @param initial_state Initial state of the engine, default is zero torque
         Engine(const engine_params& params, const engine_state& initial_state = engine_state{0.0}) ;
            
        /// *** Seteers *** ////

        /// Set the torque of the engine
        /// @param new_torque New torque value to set

        inline void set_torque(double new_torque) {
            state_.torque = new_torque;
        }

        /// Set the entire state of the engine
        /// @param new_state New state to set

        inline void set_state(const engine_state& new_state) {
            state_ = new_state;
        }

        /// Set the engine parameters
        /// @param new_params New parameters to set

         inline void set_params(const engine_params& new_params) {
            params_ = new_params;
        }

        /// *** Getters *** ////

        /// Get the current state of the engine
        /// @return Current state of the engine

        inline engine_state get_state() const { return state_; }


        /// Get the current torque of the engine
        /// @return Current torque of the engine

        inline double get_torque() const {
            return state_.torque;
        }

        /// Get the engine parameters
        /// @return Current parameters of the engine

        inline engine_params get_params() const {
            return params_;
        }


        /// *** Engine Dynamics *** ////
        
        /// Calculate the derivative of the  acutal torque given by engine  based on the engine parameters and input
        /// @param engine_input Input to the engine in range [-1 , 1 ]

        double engine_torque_derivative( double engine_input ) const ;

        /// Update the torque of the engine based on the input and time step
        /// @param engine_input Input to the engine in range [-1 , 1 ]
        /// @param dt Time step for the update

        void update_torque(double engine_input, double dt) ;

        /// *** Reset *** ////

        // Reset the engine state to the initial state
        /// @param initial_state Initial state to reset to, default is zero torque

        inline void reset(const engine_state& initial_state = engine_state{0.0}) {
            state_ = initial_state;
        }

       


    private:
        engine_params params_;
        engine_state state_;
    };

    

} // namespace metzler_model

#endif