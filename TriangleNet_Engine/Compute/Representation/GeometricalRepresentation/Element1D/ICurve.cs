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
        [Description("Returns the geometrical representation of the curve, which is a Pipe or itself, depending on the reprOptions.")] // the pipe radius corresponds to how big the Curve is when represented.
        [Input("curve", "Input curve.")]
        [Input("reprOptions", "Representation options.")]
        [Output("geom", "Geometrical representation.")]
        public static IGeometry GeometricalRepresentation(this ICurve curve, RepresentationOptions reprOptions = null)
        {
            reprOptions = reprOptions ?? new RepresentationOptions();

            if (!reprOptions.Detailed1DElements)
                return curve;

            double radius = 0.01 * reprOptions.Element1DScale;
            bool capped = reprOptions.Cap1DElements;

            return BH.Engine.Geometry.Create.Pipe(curve, radius, capped);
        }
    }
}

