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
using BH.Engine.Graphics;
using System.ComponentModel;
using BH.oM.Base.Attributes;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        // Main interface method
        [Description("Computes the Geometrical Representation of the input object and then attempts to mesh it.")]
        [Input("obj","Any object defined within BHoM.")]
        [Input("renderMeshOptions", "Options that regulate both the calculation of the Geometrical Representation and how it should be meshed.")]
        [Output("A RenderMesh, which is a geometrical mesh that can potentially have additional attributes like Colours.")]
        public static BH.oM.Graphics.RenderMesh IRenderMesh(this IObject obj, RenderMeshOptions renderMeshOptions = null)
        {
            if (obj == null) return null;

            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            RenderMesh renderMesh = null;

            // See if there is a custom BHoM mesh representation for this BHoMObject, before attempting the RenderMesh computation.
            if (obj is IBHoMObject)
                if (Query.TryGetRendermesh(obj as IBHoMObject, out renderMesh))
                    return renderMesh;

            if (obj is BH.oM.Graphics.RenderMesh)
                return obj as BH.oM.Graphics.RenderMesh;

            if (obj is RenderGeometry)
                return RenderMesh(obj as RenderGeometry);

            BH.oM.Geometry.Mesh mesh = obj as BH.oM.Geometry.Mesh;
            if (mesh != null)
                return mesh.ToRenderMesh();

            // If obj is of type IGeometry, we still need to compute its geometrical representation.
            // E.g. A BH.oM.Geometry.Point can only be represented with a Sphere, a Pixel, a Voxel, etc.
            IGeometry geomRepr = IGeometricalRepresentation(obj, renderMeshOptions.RepresentationOptions);

            if (geomRepr != null)
                renderMesh = RenderMesh(geomRepr as dynamic, renderMeshOptions);

            if (renderMesh == null)
                throw new Exception($"Could not compute the {nameof(BH.oM.Graphics.RenderMesh)} of {obj.GetType().Name}.");

            return renderMesh;
        }

        // Fallback
        private static BH.oM.Graphics.RenderMesh RenderMesh(this IGeometry geom, RenderMeshOptions renderMeshOptions = null)
        {
            throw new MissingMethodException($"Could not find a method to compute the {nameof(BH.oM.Graphics.RenderMesh)} of {geom.GetType().Name}.");
        }

    }
}

