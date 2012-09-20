//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           huniplacer.h
// Description:    convenience include
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          -
//
// License:        GNU GPL v3
//
// This file is part of huniplacer.
//
// huniplacer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// huniplacer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with huniplacer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <huniplacer/utils.h>
#include <huniplacer/crd514_kd_exception.h>
#include <huniplacer/CRD514_KD.h>
#include <huniplacer/deltarobot.h>
#include <huniplacer/imotor3.h>
#include <huniplacer/inverse_kinematics_exception.h>
#include <huniplacer/inverse_kinematics_impl.h>
#include <huniplacer/inverse_kinematics_model.h>
#include <huniplacer/measures.h>
#include <huniplacer/modbus_ctrl.h>
#include <huniplacer/modbus_exception.h>
#include <huniplacer/motion.h>
#include <huniplacer/point3.h>
#include <huniplacer/steppermotor3.h>
#include <huniplacer/motor3_exception.h>
#include <huniplacer/effector_boundaries.h>
