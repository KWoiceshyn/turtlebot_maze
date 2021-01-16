#ifndef TURTLEBOT_MAZE_PID_H
#define TURTLEBOT_MAZE_PID_H

//#include <iostream>

namespace turtlebot_maze {

    class PID {

    public:
        PID(double p, double i, double d) :
                p_gain_{p},
                i_gain_{i},
                d_gain_{d},
                i_term_max_{0.1},
                last_error_{0},
                i_term_{0},
                t_last_{0} {

        }

        double runControl(double sp, double fb, double t_now) {

            double delta_t = t_now - t_last_;

            double error = sp - fb;
            // TODO: maybe need to filter the d-term?
            double d_error = delta_t > 0.05 ? (error - last_error_) / delta_t : 0.0;
            i_term_ += error * delta_t;
            i_term_ = std::max(std::min(i_term_, i_term_max_), -i_term_max_);

            t_last_ = t_now;
            last_error_ = error;
            return p_gain_ * error + i_gain_ * i_term_ + d_gain_ * d_error;
        }

        double runControlHE(double error, double t_now, double heading_error) {

            double delta_t = t_now - t_last_;

            //double error = sp - fb;
            i_term_ += error * delta_t;
            i_term_ = std::max(std::min(i_term_, i_term_max_), -i_term_max_);

            t_last_ = t_now;
            last_error_ = error;
            //std::cout << "p " << p_gain_ * error << " i " << i_gain_ * i_term_ << " d " << d_gain_ * heading_error <<"\n";
            return p_gain_ * error + i_gain_ * i_term_ + d_gain_ * heading_error;

        }

        void reset(double t_now){
            i_term_ = 0.0;
            t_last_ = t_now;
            last_error_ = 0.0;
        }

    private:
        double i_term_;
        double last_error_;
        double t_last_;

        const double p_gain_;
        const double i_gain_;
        const double d_gain_;
        const double i_term_max_;
    };

} // namespace turtlebot_maze

#endif //TURTLEBOT_MAZE_PID_H
