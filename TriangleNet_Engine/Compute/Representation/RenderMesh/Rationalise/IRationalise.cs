/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2025, the respective contributors. All rights reserved.
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
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Rationalises the Curve into a Polyline.")]
        public static Polyline IRationalise(this ICurve curve, RenderMeshOptions renderMeshOptions = null)
        {
            if (curve is IPolyline) // no need to rationalise
                return curve.IToPolyline();

            if (curve is Line) // no need to rationalise
                return new Polyline() { ControlPoints = (curve as Line).ControlPoints() };

            return Rationalise(curve as dynamic, renderMeshOptions);
        }

        // Fallback
        private static Polyline Rationalise(this ICurve curve, RenderMeshOptions renderMeshOptions = null)
        {
            BH.Engine.Base.Compute.RecordError($"Could not find a method to rationalise the curve {curve.GetType().Name}. Currently support only Arc and Circle.");
            return null;
        }
    }
}






