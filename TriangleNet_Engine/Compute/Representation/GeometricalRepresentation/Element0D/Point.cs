/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2023, the respective contributors. All rights reserved.
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
using BH.oM.Base.Attributes;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        [Description("Returns the geometrical representation of the point, which can be a Sphere or the point itself, depending on the RepresentationOptions.")] // in the future, we might want an option to choose between sphere / box.
        [Input("point", "Point to compute the representation out from. It can be the point itself, or a Sphere that can then be meshed to represent the point location.")]
        [Input("reprOptions", "Representation options.")]
        [Input("isSubObject", "If true, do not compute the GeometricalRepresentation (return null). This is because, when isSubObject is true, we are saying that we are calling this method as part of another method, e.g. when we are computing the GeometricalRepresentation of a Line and its endpoints.\n" +
            "In such instances, we may not want to display its endpoints.")]
        [Output("geom", "Geometrical representation.")]
        public static IGeometry GeometricalRepresentation(this Point point, RepresentationOptions reprOptions = null, bool isSubObject = false)
        {
            if (isSubObject) // if it is a property of another object (e.g. a Line) do not display its endpoints.
                return null;

            reprOptions = reprOptions ?? new RepresentationOptions();

            double radius = 0.15 * reprOptions.Element0DScale;
            Sphere sphere = BH.Engine.Geometry.Create.Sphere(point, radius);

            return sphere;
        }
    } 
}

