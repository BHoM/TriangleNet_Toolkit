/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2020, the respective contributors. All rights reserved.
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

        [Description("Rationalises the Polycurve. Currently limited functionality (Arcs, or Polycurves of already linear segments).")]
        public static Polyline Rationalise(this PolyCurve curve, RenderMeshOptions renderMeshOptions = null)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            if (curve.Curves.Count == 0)
                return new Polyline();

            foreach (ICurve c in curve.SubParts())
            {
                Line line = c as Line;
                if (line != null)
                    return new Polyline() { ControlPoints = new List<Point> { line.Start, line.End } };

                Arc arc = c as Arc;
                if (arc != null)
                    return Rationalise(arc);
            }

            BH.Engine.Reflection.Compute.RecordError("Rationalisation of curves currently only supports Arcs.");
            return null;
        }

        /***************************************************/

        [Description("Rationalises the Polycurve. Currently limited functionality (Arcs, or Polycurves of already linear segments).")]
        public static Polyline Rationalise(this Arc arc, RenderMeshOptions renderMeshOptions = null)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            Polyline polyline = new Polyline();

            List<Point> controlPoints = new List<Point> { arc.IStartPoint() };

            if (arc.Radius < 0.01 && Math.Abs(arc.StartAngle - 1.5708) < 0.1)
                controlPoints.Add(arc.IEndPoint());
            else
            {
                // If not, subdivide the arc.

                int numSubdvision = (int)Math.Ceiling(Math.Abs(arc.StartAngle - 1.5708) / 1.5708);

                List<double> pointParams = Enumerable.Range(0, numSubdvision).Select(i => (double)((double)i / (double)numSubdvision)).ToList();

                controlPoints.AddRange(pointParams.Select(par => arc.IPointAtParameter(par)));
            }

            return polyline;
        }
    }
}

