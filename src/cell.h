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


#ifndef CELL_H
#define CELL_H

#include "params_structs.h"

namespace polar_grid_tracking {
    
class Cell
{
public:
    Cell();
    Cell(const double & x, const double & z, const double & sizeX, const double & sizeZ, 
         const bool & occupied, const t_Camera_params & params);
    
    void setOccupiedProb(const double & occupiedProb) { m_occupiedProb = occupiedProb; }
    
    double sigmaX() { return m_sigmaX; } 
    double sigmaZ() { return m_sigmaZ; }
    
    double occupiedProb() { return m_occupiedProb; }
    double freeProb() { return 1 - m_occupiedProb; }
    
private:
    double m_x, m_z;
    double m_sigmaX, m_sigmaZ;
    double m_sizeX, m_sizeZ;
    bool m_occupied;
    
    double m_occupiedProb;
};

}
#endif // CELL_H