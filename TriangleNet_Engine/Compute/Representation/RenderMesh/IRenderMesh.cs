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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using BH.oM.Base;


namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        // Main interface method
        public static BH.oM.Graphics.RenderMesh IRenderMesh(this IObject obj, RenderMeshOptions renderMeshOptions = null)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            if (obj is BH.oM.Graphics.RenderMesh)
                return obj as BH.oM.Graphics.RenderMesh;

            BH.oM.Geometry.Mesh mesh = obj as BH.oM.Geometry.Mesh;
            if (mesh != null)
                return (RenderMesh)mesh;

            IGeometry geomRepr = IGeometricalRepresentation(obj, renderMeshOptions.RepresentationOptions);

            if (geomRepr == null)
                return null;

            RenderMesh renderMesh = RenderMesh(geomRepr as dynamic, renderMeshOptions);

            if (renderMesh == null)
                BH.Engine.Reflection.Compute.RecordError($"Could not find a method to compute the {nameof(BH.oM.Graphics.RenderMesh)} of {obj.GetType().Name}");

            return renderMesh;
        }

        // Fallback
        private static BH.oM.Graphics.RenderMesh RenderMesh(this IGeometry geom, RenderMeshOptions renderMeshOptions = null)
        {
            return null;
        }

    } 
}