/**
(C) Copyright 2011-2025 DQ Robotics Developers

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
#pragma once
#include <dqrobotics/DQ.h>
#include <string>
#include <vector>

using namespace Eigen;

namespace DQ_robotics
{
class DQ_CoppeliaSimRobot
{
public:
    virtual ~DQ_CoppeliaSimRobot() = default;
    virtual void set_configuration(const VectorXd& configuration) = 0;
    virtual void set_target_configuration(const VectorXd& target_configuration) = 0;
    virtual  VectorXd get_configuration() = 0;
    virtual void set_target_configuration_velocities(const VectorXd& target_configuration_velocities) = 0; 
    virtual  VectorXd get_configuration_velocities() = 0;
    virtual void set_target_configuration_forces(const VectorXd& target_configuration_forces) = 0; 
    virtual  VectorXd get_configuration_forces() = 0;
protected:
    DQ_CoppeliaSimRobot() = default;
};

}
