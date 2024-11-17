/**
(C) Copyright 2024 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

DQ Robotics website: dqrobotics.github.io

Contributors:

   1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
        - Responsible for the original implementation.
*/

#include <dqrobotics/DQ.h>
#include <string>
#include <vector>

using namespace Eigen;

namespace DQ_robotics
{
class DQ_CoppeliaSimInterface
{
protected:
    DQ_CoppeliaSimInterface() = default;
public:
    virtual ~DQ_CoppeliaSimInterface() = default;
    virtual bool connect(const std::string& host, const int& port, const int&TIMEOUT_IN_MILISECONDS) = 0;
    virtual void trigger_next_simulation_step() const = 0;
    virtual void set_stepping_mode(const bool& flag) const = 0;
    virtual void start_simulation() const = 0;
    virtual void stop_simulation()  const = 0;

    virtual int get_object_handle(const std::string& objectname) = 0;
    virtual std::vector<int> get_object_handles(const std::vector<std::string>& objectnames) = 0;

    virtual DQ   get_object_translation(const std::string& objectname) = 0;
    virtual void set_object_translation(const std::string& objectname, const DQ& t) = 0;

    virtual DQ   get_object_rotation   (const std::string& objectname) = 0;
    virtual void set_object_rotation   (const std::string& objectname, const DQ& r) = 0;

    virtual DQ   get_object_pose       (const std::string& objectname) = 0;
    virtual void set_object_pose       (const std::string& objectname, const DQ& h) = 0;

    virtual VectorXd get_joint_positions(const std::vector<std::string>& jointnames) = 0;
    virtual void     set_joint_positions(const std::vector<std::string>& jointnames, const VectorXd& joint_positions) = 0;
    virtual void     set_joint_target_positions(const std::vector<std::string>& jointnames, const VectorXd& joint_target_positions) = 0;

    virtual VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) = 0;
    virtual void     set_joint_target_velocities(const std::vector<std::string>& jointnames, const VectorXd& joint_target_velocities) = 0;

    virtual void     set_joint_torques(const std::vector<std::string>& jointnames, const VectorXd& torques) = 0;
    virtual VectorXd get_joint_torques(const std::vector<std::string>& jointnames) = 0;

};

}
