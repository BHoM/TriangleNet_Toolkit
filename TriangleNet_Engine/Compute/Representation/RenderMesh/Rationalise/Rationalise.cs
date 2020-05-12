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

        [Description("Rationalises the Polycurve into a Polyline. Currently limited functionality.")]
        public static Polyline Rationalise(this PolyCurve curve, RenderMeshOptions renderMeshOptions = null, int minSubdivisions = 3)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            if (curve.Curves.Count == 0)
                return new Polyline();

            Polyline polyline = new Polyline();
            polyline.ControlPoints.Add(curve.SubParts()[0].IStartPoint());

            foreach (ICurve c in curve.SubParts())
            {
                Line line = c as Line;
                if (line != null)
                {
                    polyline.ControlPoints.Add(line.End);
                    continue;
                }

                Polyline rationalised = Rationalise(c as dynamic, renderMeshOptions, minSubdivisions);

                List<Point> points = rationalised.ControlPoints.Skip(1).ToList();

                polyline.ControlPoints.AddRange(points);
            }

            if (polyline == null || polyline.ControlPoints.Count < 2)
                BH.Engine.Reflection.Compute.RecordError("Rationalisation of curves currently only supports Arcs.");

            return polyline;
        }

        /***************************************************/

        [Description("Rationalises the Arc into a Polyline.")]
        public static Polyline Rationalise(this Arc arc, RenderMeshOptions renderMeshOptions = null, int minSubdivisions = 3)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            Polyline polyline = new Polyline();

            List<Point> controlPoints = new List<Point> { arc.IStartPoint() };

            if (arc.Radius < 0.01 && Math.Abs(arc.StartAngle - 1.5708) < 0.1)
                controlPoints.Add(arc.IEndPoint());
            else
            {
                // If not, subdivide the arc.
                double arcAngle = Math.Abs(Math.Abs(arc.StartAngle - arc.EndAngle));
                int numSubdvision = (int)(Math.Ceiling(1.5708 / (arcAngle - 1.5708)) * renderMeshOptions.Element1DRefinement) - 1;

                // Scale the number of subdivisions based on the Options
                numSubdvision = (int)Math.Ceiling(numSubdvision * renderMeshOptions.Element1DRefinement);

                // Check the number of subdivisions is over the minimum acceptable
                numSubdvision = numSubdvision < minSubdivisions ? minSubdivisions : numSubdvision;

                List<double> pointParams = Enumerable.Range(0, numSubdvision).Select(i => (double)((double)i / (double)numSubdvision)).ToList();

                controlPoints.AddRange(pointParams.Select(par => arc.IPointAtParameter(par)));
            }

            polyline.ControlPoints = controlPoints;

            return polyline;
        }

        [Description("Rationalises the Circle into a Polyline.")]
        public static Polyline Rationalise(this Circle circle, RenderMeshOptions renderMeshOptions = null, int minSubdivisions = 3)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            Polyline polyline = new Polyline();

            List<Point> controlPoints = new List<Point> { circle.IStartPoint() };

            // Subdivide the circle.
            // Empyrical formula to extract a reasonable amount of segments
            int numSubdvision = (int)(Math.Ceiling(circle.Radius * 10) * renderMeshOptions.Element1DRefinement);

            // Scale the number of subdivisions based on the Options
            numSubdvision = (int)Math.Ceiling(numSubdvision * renderMeshOptions.Element1DRefinement);
            
            // Check the number of subdivisions is over the minimum acceptable
            numSubdvision = numSubdvision < minSubdivisions ? minSubdivisions : numSubdvision;

            List<double> pointParams = Enumerable.Range(0, numSubdvision).Select(i => (double)((double)i / (double)numSubdvision)).ToList();
            pointParams.Add(1);

            controlPoints.AddRange(pointParams.Select(par => circle.IPointAtParameter(par)));

            polyline.ControlPoints = controlPoints;

            return polyline;
        }
    }
}
