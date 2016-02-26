//Copyright (C) 2013 Sergio E. Gonzalez and Emiliano D. González
//Facultad de Ciencias Exactas y Naturales, Universidad de Buenos Aires, Buenos Aires, Argentina.
// 
//C/C++, Java and XML/YML code for EKF SLAM from a monocular sequence.
//
//This file is part of OpenEKFMonoSLAM.
//
//OpenEKFMonoSLAM is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//OpenEKFMonoSLAM is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with OpenEKFMonoSLAM.  If not, see <http:www.gnu.org/licenses/>.
//
//If you use this code for academic work, please reference:
//GONZALEZ, S.; E. GONZÁLEZ; M. NITSCHE; P. DE CRISTÓFORIS. "Odometría Visual para Robots Móviles Utilizando Smartphones como Unidad de Sensado y Procesamiento". En: Jornadas Argentinas de Robótica, 8as : 2014 : Ciudad Autónoma de Buenos Aires. Actas : VIII Jornadas Argentinas de Robótica . (8 : Ciudad Autónoma de Buenos Aires, 12-14 de noviembre 2014).
//
//Authors:    Sergio E. Gonzalez - segonzalez@dc.uba.ar
//            Emiliano D. González - edgonzalez@dc.uba.ar
//
//Departamento de Computación
//Facultad de Ciencias Exactas y Naturales
//Universidad de Buenos, Buenos Aires, Argentina
//Date: June 2013
#include "stdafx.h"

#include "ConfigurationDataReader.h"

// ------------------------------------------------------------------------------------------
// Public Methods Implementation
// ------------------------------------------------------------------------------------------

bool ConfigurationDataReader::loadConfigurationFromFileNode(const cv::FileNode &dataFileNode)
{
    bool success = true;
    
    cv::FileNodeIterator itEnd = dataFileNode.end();
    for (cv::FileNodeIterator it = dataFileNode.begin(); it != itEnd && success; it++)
    {
        const char *newDataValueName = (*it).name().c_str();
        success = success && loadNodeFromFileNode(dataFileNode, newDataValueName);
    }
    
    return success;
}

// ------------------------------------------------------------------------------------------

const void* ConfigurationDataReader::getValueForAlias(const char *alias) const
{
    Dictionary< void* >::const_iterator it = _dataMap.find(alias);
    
    if (it == _dataMap.end())
    {
        return NULL;
    }
    
    return it->second;
}

// ------------------------------------------------------------------------------------------