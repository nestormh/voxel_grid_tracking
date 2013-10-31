/*
 *  Copyright 2013 Néstor Morales Hernández <nestor@isaatc.ull.es>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "cell.h"

namespace polar_grid_tracking {
    
// FIXME: Is this value constant? Where do I get it from?
#define DISPARITY_COMPUTATION_ERROR 0.25

Cell::Cell() 
{
    
}

Cell::Cell(const double & x, const double & z, const double & sizeX, const double & sizeZ, 
           const bool & occupied, const t_Camera_params & params) : 
                m_x(x), m_z(z), m_sizeX(sizeX), m_sizeZ(z), m_occupied(occupied)
{
    const double sigmaZ = (m_z * m_z * DISPARITY_COMPUTATION_ERROR) / (params.baseline * params.ku);
    const double sigmaX = (m_x * m_x * sigmaZ) / m_z;
    
    m_sigmaZ = sigmaZ / m_sizeZ;
    m_sigmaX = sigmaX / m_sizeX;
    
}
    
}