/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2022, the respective contributors. All rights reserved.
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
using BH.oM.Reflection.Attributes;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Returns the RenderMesh for the given object, that is a mesh that can be used for Graphical Display.")]
        [Input("planarSurface", "Input planarSurface.")]
        [Input("renderMeshOptions", "Input renderMeshOptions for how the RenderMesh is computed.")]
        [Output("renderMesh", "Resulting RenderMesh.")]
        public static BH.oM.Graphics.RenderMesh RenderMesh(this PlanarSurface planarSurface, RenderMeshOptions renderMeshOptions = null)
        {
            if (planarSurface == null)
            {
                BH.Engine.Reflection.Compute.RecordError("Cannot compute the mesh of a null planar surface.");
                return null;
            }

            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            Polyline externalBoundary = planarSurface.ExternalBoundary.IRationalise(renderMeshOptions);
            if (externalBoundary == null)
            {
                BH.Engine.Reflection.Compute.RecordError($"Meshing for {nameof(PlanarSurface)} works only if the {nameof(planarSurface.ExternalBoundary)} is of type {nameof(Polyline)}");
                return null;
            }

            List<Polyline> internalBoundaries = planarSurface.InternalBoundaries.Select(c => c.IRationalise(renderMeshOptions)).ToList();

            if (internalBoundaries.Count != internalBoundaries.Count)
            {
                BH.Engine.Reflection.Compute.RecordError($"Meshing for {nameof(PlanarSurface)} works only if all of the {nameof(planarSurface.InternalBoundaries)} are of type {nameof(Polyline)}");
                return null;
            }

            List<Polyline> polylines = BH.Engine.Geometry.Triangulation.Compute.DelaunayTriangulation(externalBoundary, internalBoundaries);
            List<RenderMesh> singleFacesMeshes = new List<RenderMesh>();

            foreach (Polyline poly in polylines)
            {
                RenderMesh singleFaceMesh = new RenderMesh();

                singleFaceMesh.Vertices.AddRange(poly.ControlPoints.Select(p => (RenderPoint)p));
                singleFaceMesh.Faces.Add(new Face() { A = 0, B = 1, C = 2 });

                singleFacesMeshes.Add(singleFaceMesh);
            }

            return singleFacesMeshes.JoinRenderMeshes();
        }
    }
}

