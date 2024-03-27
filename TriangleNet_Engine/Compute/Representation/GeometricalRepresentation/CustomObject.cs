/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2024, the respective contributors. All rights reserved.
 *
 * Each contributor holds copyright over their respective contributions.
 * The project versioning (Git) records all such contribution source information.
 *                                           
 *                                                                              
 * The BHoM is free software: you can redistribute it and/or modify         
 * it under the terms of the GNU Lesser General Public License as published by  
 * the Free Software Foundation, either version 3.0 of the License, or          
 * (at your option) any later version.                                          
 *                                                                              
 * The BHoM is distributed in the hope that it will be useful,              
 * but WITHOUT ANY WARRANTY; without even the implied warranty of               
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 
 * GNU Lesser General Public License for more details.                          
 *                                                                            
 * You should have received a copy of the GNU Lesser General Public License     
 * along with this code. If not, see <https://www.gnu.org/licenses/lgpl-3.0.html>.      
 */

using BH.oM.Geometry;
using BH.oM.Graphics;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using BH.oM.Base;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        // Required because of weird behaviour of IGeometry() on CustomObjects. See below.
        private static IGeometry GeometricalRepresentation(this CustomObject obj, RepresentationOptions reprOptions = null)
        {
            IGeometry geometricalRepresentation = null;

            if (obj != null)
                geometricalRepresentation = BH.Engine.Base.Query.IGeometry(obj);

            // Weirdly, by default, IGeometry() on a CustomObject always returns an empty CompositeGeometry.
            // This would generate a lot of needed checks downhill; so instead, return null.
            if (geometricalRepresentation != null && (geometricalRepresentation as CompositeGeometry).Elements.Count < 1)
                geometricalRepresentation = null;

            return geometricalRepresentation;
        }
    }
}



