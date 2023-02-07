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
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Returns the RenderMesh for the given object, that is a mesh that can be used for Graphical Display.")]
        [Input("extrusion", "Input extrusion.")]
        [Input("renderMeshOptions", "Input renderMeshOptions for how the RenderMesh is computed.")]
        [Output("renderMesh", "Resulting RenderMesh.")]
        public static BH.oM.Graphics.RenderMesh RenderMesh(this Extrusion extrusion, RenderMeshOptions renderMeshOptions = null)
        {
            if (extrusion == null)
            {
                BH.Engine.Base.Compute.RecordError("Cannot compute the mesh of a null extrusion.");
                return null;
            }

            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            Line line = extrusion.Curve as Line;
            Polyline polyline = extrusion.Curve as Polyline;

            if (line == null && polyline == null)
                polyline = Compute.IRationalise(extrusion.Curve, renderMeshOptions); // try to rationalise the extrusion profile

            if (line == null && polyline == null)
            {
                BH.Engine.Base.Compute.RecordError($"Calling RenderMesh for {nameof(Extrusion)} currently works only if the {nameof(Extrusion.Curve)} is composed of linear segments.");
                return null;
            }

            List<Face> faces = new List<Face>();
            List<Point> points = new List<Point>();

            if (line != null)
            {
                points.Add(new Point() { X = line.Start.X, Y = line.Start.Y, Z = line.Start.Z });
                points.Add(new Point() { X = line.End.X, Y = line.End.Y, Z = line.End.Z });
                points.Add(new Point() { X = line.End.X + extrusion.Direction.X, Y = line.End.Y + extrusion.Direction.Y, Z = line.End.Z + extrusion.Direction.Z });
                points.Add(new Point() { X = line.Start.X + extrusion.Direction.X, Y = line.Start.Y + extrusion.Direction.Y, Z = line.Start.Z + extrusion.Direction.Z });

                faces.Add(new Face() { A = 0, B = 1, C = 2, D = 3 });

                return new RenderMesh() { Faces = faces, Vertices = points.Cast<RenderPoint>().ToList() };
            }

            RenderMesh bottomCap = null;
            RenderMesh topCap = null;

            if (polyline != null)
            {
                points.AddRange(polyline.ControlPoints);
                points.AddRange(polyline.ControlPoints.Select(p => new Point() { X = p.X + extrusion.Direction.X, Y = p.Y + extrusion.Direction.Y, Z = p.Z + extrusion.Direction.Z }));

                int faceNo = polyline.ControlPoints.Count() - 1;

                for (int i = 0; i < faceNo; i++)
                {
                    int add = 0;//extrusion.Curve.IIsClosed() ? 1 : 0;

                    faces.Add(new Face() { A = i, B = i + 1, C = polyline.ControlPoints.Count() + (i + 1) + add, D = polyline.ControlPoints.Count() + i + add });
                }

                if (extrusion.Curve.IIsClosed() && extrusion.Capped)
                {
                    Polyline bottomBoundary = extrusion.Curve.IRationalise(renderMeshOptions);

                    if (bottomBoundary == null)
                        Base.Compute.RecordNote("Mesh of Extrusion caps still not implemented for extrusion defined with a curve of this type.");
                    else
                    {
                        bottomCap = Create.PlanarSurface(bottomBoundary).RenderMesh(renderMeshOptions);

                        Polyline topBoundary = bottomBoundary.Translate(extrusion.Direction);
                        topCap = Create.PlanarSurface(topBoundary).RenderMesh(renderMeshOptions);
                    }
                }

                RenderMesh renderMesh = new RenderMesh() { Vertices = points.Select(pt => (RenderPoint)pt).ToList(), Faces = faces };

                if (bottomCap != null)
                    renderMesh = renderMesh.JoinRenderMeshes(bottomCap);

                if (topCap != null)
                    renderMesh = renderMesh.JoinRenderMeshes(topCap);

                return renderMesh;
            }

            BH.Engine.Base.Compute.RecordError($"Calling RenderMesh for {nameof(Extrusion)} currently works only if the {nameof(Extrusion.Curve)} is composed of linear segments.");
            return null;
        }
    }
}

